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

/*
 * Stack for the event-drain thread. The dispatch it runs is not shallow -
 * handle_event -> depcmd -> LOG_INF, and the periodic stats line alone passes
 * about forty arguments through cbprintf on this stack, so 512 B overflows.
 * Remaining headroom is reported as "evtstack" in the periodic stats line.
 */
#define UDC_DWC3_EVT_STACK_SIZE 1536
/*
 * Priority of the event-drain thread. Cooperative, so a drain pass is not
 * chopped up by preemptible work, and placed below the two threads it must
 * never starve and above the one that must never delay it
 * (CONFIG_NUM_COOP_PRIORITIES is 16).
 */
#define UDC_DWC3_EVT_THREAD_PRIO K_PRIO_COOP(6)


/* TRB memory buffer fields */
#define UDC_DWC3_TRB_STATUS_BUFSIZ_MASK				GENMASK(23, 0)
#define UDC_DWC3_TRB_STATUS_PCM1_MASK				GENMASK(25, 24)
/*
 * Short Packet Received. Set by the controller when it writes a TRB back on an
 * OUT transfer to mark the last TRB used for that transfer descriptor; it is
 * bit 26 of the {TRBSTS, Rsvd, SPR, PCM1, BUFSIZ} status dword.
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
 * Stream ID / SOF Number. PCM1 and SPR are in the TRB status dword, not this
 * control word (Figure 3-1). The control word holds only HWO, LST, CHN, CSP,
 * TRBCTL, ISP/IMI, IOC and this field; bits 13:12 and 31:30 are reserved.
 */
#define UDC_DWC3_TRB_CTRL_SIDSOFN_MASK				GENMASK(29, 14)

/* Incomplete coverage of all fields, but suited for what this driver supports */
#define UDC_DWC3_EVT_MASK					GENMASK(11, 0)
#define UDC_DWC3_DEPEVT_EPN_MASK				GENMASK(5, 1)
/*
 * Fields the controller returns in an Endpoint Command Complete event.
 * Programming Guide 3.30b, Table 3-7 "Device Endpoint-n Events: DEPEVT":
 */
#define UDC_DWC3_DEPEVT_CMDTYP_MASK				GENMASK(27, 24)
#define UDC_DWC3_DEPEVT_XFERRSCIDX_MASK				GENMASK(22, 16)
#define UDC_DWC3_DEPEVT_CMDSTATUS_MASK				GENMASK(15, 12)
#define UDC_DWC3_DEPEVT_XFERCOMPLETE(epn)			(((epn) << 1) | (0x01 << 6))
#define UDC_DWC3_DEPEVT_XFERINPROGRESS(epn)			(((epn) << 1) | (0x02 << 6))
#define UDC_DWC3_DEPEVT_XFERNOTREADY(epn)			(((epn) << 1) | (0x03 << 6))
#define UDC_DWC3_DEPEVT_RXTXFIFOEVT(epn)			(((epn) << 1) | (0x04 << 6))
#define UDC_DWC3_DEPEVT_STREAMEVT(epn)				(((epn) << 1) | (0x06 << 6))
#define UDC_DWC3_DEPEVT_EPCMDCMPLT(epn)				(((epn) << 1) | (0x07 << 6))
/*
 * SPEC, Programming Guide 3.30b, DEPEVT field 15:12 "Event Status", within an
 * XferNotReady event (p.326):
 */
#define UDC_DWC3_DEPEVT_STATUS_CONTROL_MASK			GENMASK(13, 12)
#define UDC_DWC3_DEPEVT_STATUS_CONTROL_SETUP			(0x0 << 12)
#define UDC_DWC3_DEPEVT_STATUS_CONTROL_DATA			(0x1 << 12)
#define UDC_DWC3_DEPEVT_STATUS_CONTROL_STATUS			(0x2 << 12)
/*
 * Event Status is bits 15:12 of every endpoint-specific event and its meaning
 * depends on the event type. The per-type decodes below, and the control-stage
 * mask above, all live in that field.
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
 * Event Status has no bus-error bit; the databook's "bus error" is the
 * GBUSERRADDR SoC-bus registers, unrelated to endpoint events. Defined as 0 so
 * that a reference still builds and a test on it reads false instead of
 * sampling an endpoint-number bit.
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
 * link state at the time of the event, in the same encoding as DSTS.
 */
#define UDC_DWC3_DEVT_EVTINFO_LINKSTATE_MASK			GENMASK(19, 16)
#define UDC_DWC3_DEVT_EVTINFO_SS				BIT(20)
/*
 * Sampling periods for the rate-limited diagnostics, both prime. The ring holds
 * CONFIG_UDC_DWC3_EVENTS_NUM (16) entries, so a period that is a multiple of 16
 * samples the same slot every time when the sampled thing advances by a
 * constant stride. Keep these coprime with the ring size.
 */
/* One line per this many repeats of the same link state; see
 * udc_dwc3_log_link_event().
 */
#define UDC_DWC3_EVT_LINK_LOG_EVERY				257u
/*
 * Heartbeat tick, bounding how long the event drain can sit unscheduled.
 */
#define UDC_DWC3_HEARTBEAT_MS					200u
/*
 * How many heartbeat beats between event-ring statistics lines. At
 * UDC_DWC3_HEARTBEAT_MS this is a wall-clock period, so the report keeps
 * coming while the event ring is stalled - which is when it matters.
 */
#define UDC_DWC3_EVT_STATS_BEATS 25u

/*
 * Beats between CORE debug-register samples (25 x 200 ms = 5 s). Several of
 * these registers mean nothing without a healthy baseline to compare a wedge
 * dump against, so they are sampled periodically. Costs ~0.2 lines/s.
 */
#define UDC_DWC3_CORE_DBG_BEATS					25u


/*
 * Kick the event handler if it has not completed a pass within this long while
 * the controller still says events are outstanding. This is the backstop for a
 * drain that is not running at all, not a request deadline. The lower bound is
 * the driver's own console cost: the stats line is emitted inside a drain pass
 * and LOG_MODE_MINIMAL makes it a synchronous uart_poll_out, ~250 chars at
 * 115200 = ~22 ms; 100 ms clears any plausible pass and is 50x inside the
 * host's 5 s control timeout.
 */
#define UDC_DWC3_EVT_IDLE_KICK_MS				100u

/*
 * How old a give-up run must be before the heartbeat reports it. A second, so
 * the report rate does not track the heartbeat tick.
 */
#define UDC_DWC3_EVT_GAVEUP_AGE_MS				1000u

/*
 * How long control traffic may stop before the driver says so.
 */
#define UDC_DWC3_CTRL_QUIET_MS					2000u

/*
 * The point at which the ring has no room left to give. Programming Guide
 * 3.30b: "The controller always leaves one entry free in each Event Buffer",
 * so the fullest GEVNTCOUNT can report is one entry short of the ring.
 */
#define UDC_DWC3_EVT_RING_FULL_BYTES				\
	(((CONFIG_UDC_DWC3_EVENTS_NUM) - 1u) * sizeof(uint32_t))

/*
 * How long a head slot must stay unreadable, with the ring full, before the
 * drain gives up on it and skips it.
 */
#define UDC_DWC3_EVT_SKIP_AFTER_MS				250u

/*
 * How long the heartbeat must see GEVNTCOUNT > 0 with nothing handled before it
 * calls the drain stuck.
 */
#define UDC_DWC3_HB_DRAIN_STUCK_MS				(3u * UDC_DWC3_HEARTBEAT_MS)
/*
 * Recovery FSM thresholds. SUSPECT_MS is how long an anomaly must persist
 * before any action; it sits above the normal service latency of the thing
 * watched - a descriptor legitimately stays HWO=1 until the host asks for it -
 * and well inside the host's 5 s control timeout. CMDCMPLT_MS bounds an
 * outstanding Command Complete; MAX_ACTIONS caps the End Transfer + Start
 * Transfer recoveries one wedge episode may get.
 */
#define UDC_DWC3_RECOV_SUSPECT_MS				200u
#define UDC_DWC3_RECOV_CMDCMPLT_MS			(3u * UDC_DWC3_HEARTBEAT_MS)
#define UDC_DWC3_RECOV_MAX_ACTIONS				2u
/*
 * How long R_UNFETCHED must persist before it counts as a fault.
 */
#define UDC_DWC3_RECOV_UNFETCHED_MS				2000u
/*
 * The same observation on EP0-IN, which cannot wait that long: a control
 * transfer runs on the host's clock, and the host retries, fails the request
 * and resets the device well inside two seconds.
 */
#define UDC_DWC3_RECOV_CTRL_UNFETCHED_MS			250u
/*
 * Full fault reports emitted per endpoint per boot.
 */
#define UDC_DWC3_RECOV_REPORT_MAX				3u


/*
 * Cumulative give-ups on one slot that prove it dead regardless of wall clock.
 * The 1000 ms route needs an uninterrupted run, which anything resetting
 * drain.attempts can starve; this many looks that all found the slot empty is
 * conclusive on its own and gives the skip a reset-resistant second path.
 */
#define UDC_DWC3_EVT_DEAD_SLOT_GIVEUPS				64u

/*
 * Minimum age of a give-up run before any abort route may discard a slot.
 */
#define UDC_DWC3_EVT_DEAD_SLOT_MIN_MS				200u

#define UDC_DWC3_EVT_DEAD_SLOT_MS				1000u

/*
 * Minimum age before the drain may act on the look-ahead proof rather than on
 * the timeout above.
 */
#define UDC_DWC3_EVT_LOOKAHEAD_MIN_MS				50u

/*
 * Defined: the heartbeat may act on a dead slot. Undefined: it only reports it.
 */
#define UDC_DWC3_EVT_DEAD_SLOT_RECOVER

/*
 * Print the eight setup bytes of every control transfer.
 */
#define UDC_DWC3_LOG_EVERY_SETUP

/*
 * Escalate a stuck SETUP to a core soft reset when the Set Stall that normally
 * clears it has failed twice running.
 */
#define UDC_DWC3_SETUP_STUCK_RESET

/*
 * How long a slot must stay empty before the event write is presumed lost
 * rather than late.
 */
#define UDC_DWC3_EVT_MISSED_MS					1000u

/*
 * Hardware state dump on a lost event write, for the RTL side.
 */
#define STALL_DIAG_LOG

/*
 * What a consumed - or never yet written - event slot holds. 0xFFFFFFFF cannot
 * be a real event of either class: bit 0 = 1 makes it non-endpoint-specific,
 * and a device event requires bits 7:1 to be 7'h00 where this has 0x7f. Zero
 * is not used because it decodes as a well-formed endpoint event (physical
 * endpoint 0, event type 4'h0, which is Reserved rather than impossible on a
 * re-implemented core).
 */
#define UDC_DWC3_EVT_CONSUMED_ENTRY_VALUE			0xFFFFFFFFu
#define UDC_DWC3_DISPATCH_STUCK_MS				250u
/*
 * Consecutive refused event-ring acknowledgements before the ring is declared
 * untrustworthy and the controller is recovered.
 */
#define UDC_DWC3_EVT_ACK_DEAD_MAX				3u
#define UDC_DWC3_DEVT_VNDRDEVTSTRCVED				(BIT(0) | (0xc << 8))

/* Device Endpoint Commands and Parameters */
#define UDC_DWC3_DEPCMDPAR2(n)					(0xc800 + 16 * (n))
#define UDC_DWC3_DEPCMDPAR1(n)					(0xc804 + 16 * (n))
#define UDC_DWC3_DEPCMDPAR0(n)					(0xc808 + 16 * (n))
#define UDC_DWC3_DEPCMD(n)					(0xc80c + 16 * (n))
/* Common fields to DEPCMD */
#define UDC_DWC3_DEPCMD_HIPRI_FORCERM				(1 << 11)
/*
 * Command Interrupt On Completion: asks the controller to raise an Endpoint
 * Command Complete event (DEPEVT_EPCMDCMPLT) once the command has finished.
 * Required on End Transfer, where CmdAct clearing means only that the command
 * was accepted while the databook says the controller "will wait until it can
 * complete operations for the endpoint before returning the Command Complete
 * event" - the only signal that DMA has stopped before a fresh Start Transfer.
 */
#define UDC_DWC3_DEPCMD_CMDIOC					BIT(8)
#define UDC_DWC3_DEPCMD_STATUS_MASK				GENMASK(15, 12)
#define UDC_DWC3_DEPCMD_STATUS_OK				(0 << 12)
#define UDC_DWC3_DEPCMD_STATUS_CMDERR				(1 << 12)
#define UDC_DWC3_DEPCMD_XFERRSCIDX_MASK				GENMASK(22, 16)
/*
 * Not a transfer resource index: returned by udc_dwc3_depcmd() when a command
 * fails, and held in ep_data->xferrscidx while none is established. XferRscIdx
 * is a 7-bit field (DEPCMD/DEPEVT [22:16]), so no index the controller can
 * assign collides with this value and no separate flag is needed.
 */
#define UDC_DWC3_XFERRSCIDX_INVALID				0xffffffffU
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
/*
 * AXI Pipelined Transfers Burst Request Limit. Encoded as N-1, so 0x0 is one
 * outstanding request and 0xf is sixteen: "when the AXI master reaches this
 * limit, it does not make any more requests on the AXI ARADDR and AWADDR buses
 * until the associated data phases complete".
 */
#define UDC_DWC3_GSBUSCFG1_PIPETRANSLIMIT_MASK			GENMASK(11, 8)
/* Break DMA transfers at the 1k page boundary instead of 4k. */
#define UDC_DWC3_GSBUSCFG1_EN1KPAGE				BIT(12)
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
/* Bits 10, 11 and 13 are RESERVED in this controller - do not write them. */
#define UDC_DWC3_DEVTEN_VNDRDEVTSTRCVEDEN			BIT(12)
#define UDC_DWC3_DEVTEN_ERRTICERREN				BIT(9)
#define UDC_DWC3_DEVTEN_SOFEN					BIT(7)
#define UDC_DWC3_DEVTEN_U3L2L1SUSPEN				BIT(6)
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
 * Note what that permits and what it does not. It licenses ONE read per
 * assertion, taken before processing. It does not license re-reading the
 * register after acknowledging and treating the result as a fresh count: by the
 * databook's own statement that value may still reflect events already handed
 * back. See udc_dwc3_evt_drain(), which reads it exactly once.
 */
#define UDC_DWC3_GEVNTCOUNT_MASK				GENMASK(15, 0)
#define UDC_DWC3_GEVNTCOUNT_EVNT_HANDLER_BUSY			BIT(31)

/*
 * DWC_usb3 Programming Guide 3.30b, section 1.3.13 DEV_IMOD[0], Table 1-90
 * (p.253): bits 15:0 DEVICE_IMODI (Interrupt Moderation Interval), bits 31:16
 * DEVICE_IMODC (down counter).
 */
#define UDC_DWC3_DEV_IMOD(n)					(0xca00 + 4 * (n))
#define UDC_DWC3_DEV_IMOD_DEVICE_IMODI_MASK			GENMASK(15, 0)
#define UDC_DWC3_DEV_IMOD_DEVICE_IMODC_MASK			GENMASK(31, 16)
/* 250 ns per unit (Table 1-90), so 1 ms = 4000. */
#define UDC_DWC3_DEV_IMOD_INTERVAL_1MS				4000U

/*
 * GEVNTCOUNT, read with a full fence.
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
 * Acknowledge `words` event words. The only write path to GEVNTCOUNT once the
 * ring is live, and the drain pass calls it exactly once. EVNT_HANDLER_BUSY
 * (bit 31) goes out with the count - Table 1-68. words == 0 is legal: it
 * credits nothing and clears the handler-busy bit.
 */
static inline void udc_dwc3_gevntcount_ack(const mm_reg_t base,
					   const uint32_t words)
{
	sys_write32((words * sizeof(uint32_t)) |
		    UDC_DWC3_GEVNTCOUNT_EVNT_HANDLER_BUSY,
		    base + UDC_DWC3_GEVNTCOUNT(0));
}

/*
 * Release the event buffer to the controller. A bare zero, with
 * EVNT_HANDLER_BUSY CLEAR - not an acknowledgement of zero words, which sets
 * bit 31 and means the opposite.
 */
static inline void udc_dwc3_gevntcount_enable(const mm_reg_t base)
{
	sys_write32(0, base + UDC_DWC3_GEVNTCOUNT(0));
}

/* USB Device Active USB Endpoint Enable */
/* Global Device TX FIFO DMA Priority: bit[n] = 1 gives TXFIFO[n] high priority. */
#define UDC_DWC3_GTXFIFOPRIDEV					0xc610

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
/*
 * How many physical endpoints this driver keeps its own per-endpoint copies for.
 */
#define UDC_DWC3_MAX_EPN					16U

/*
 * Endpoint excluded from the per-arm TRB trace below.
 */
#define UDC_DWC3_TRBLOG_SKIP_EP					0x85U

#define UDC_DWC3_GDBGFIFOSPACE					0xc160
#define UDC_DWC3_GDBGFIFOSPACE_AVAILABLE_MASK			GENMASK(31, 16)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_MASK			GENMASK(8, 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_TXFIFO			(0x0 << 5)
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
/* Bounded CSR READS, not microseconds - see udc_dwc3_dgcmd_wait_idle(). */
#define UDC_DWC3_DGCMD_POLL_MAX					200u
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
 */
#define _EPN_IS_VALID(cfg, epn) \
	(((epn) & 1) ? ((uint32_t)((epn) >> 1) < (cfg)->num_in_eps) \
		     : ((uint32_t)((epn) >> 1) < (cfg)->num_out_eps))
#define _NUM_FIFO_SPACE 16
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
	 * alignment TRB at.
	 */
	/* USB device configuration */
	int maximum_speed_idx;
	/* Pointers to event buffer fetched by DWC3 with DMA */
	volatile uint32_t *evt_buf;
	/* Data used by vendor-specific functions ("quirks") */
	const void *quirk_config;
	void *quirk_data;
	/* IRQ management functions */
	void (*irq_connect_func)(void);
	void (*irq_enable_func)(void);
	void (*irq_disable_func)(void);
	/* Number of hardware endpoint set for input or output */
	uint8_t num_in_eps;
	uint8_t num_out_eps;
};

/*
 * RECOVERY STATE MACHINE. One enum, one per-endpoint context, one function.
 *
 * THE RULE: a recovery may only read state that SURVIVES EVENT LOSS. The fault
 * is the controller counting an event in GEVNTCOUNT and never writing it, so
 * anything keyed off "the event told me" cannot fire in the case that needs it.
 * Every state is entered on evidence from the TRB ring, the endpoint registers
 * or the driver's own bookkeeping - never an event.
 *
 * INVARIANTS:
 *   1. Only udc_dwc3_recover_step() writes recov.state.
 *   2. It runs from ONE site, the heartbeat, under the UDC mutex. The
 *      stuck-claim recovery and SETUP watchdog also run there, but stand down
 *      while this machine has an episode open on EP0.
 *   3. At most ONE controller action per call; a failed action shows up as the
 *      state not advancing.
 *   4. Every state is timed (recov.t0) and bounded (recov.attempts).
 *   5. IDLE is re-entered ONLY on a retire, never on a timer.
 *   6. A discarded event is DATA, not a state write: the drain bumps a counter
 *      and this function reopens the episode under the mutex.
 */
enum udc_dwc3_recov_state {
	UDC_DWC3_RECOV_IDLE = 0,	/* nothing wrong that durable state shows  */
	UDC_DWC3_RECOV_SUSPECT,		/* anomaly seen once; timing it            */
	UDC_DWC3_RECOV_RESYNC,		/* O2: controller finished, sw never popped */
	UDC_DWC3_RECOV_RECLAIM,		/* O3: controller holds a descriptor       */
	UDC_DWC3_RECOV_RESTART,		/* O4/O5: transfer resource gone           */
	UDC_DWC3_RECOV_STALL,		/* O1: control stage never armed           */
	UDC_DWC3_RECOV_FAILED,		/* budget spent; report once, then hands off */
};

/*
 * THE TRANSFER STATE OF ONE ENDPOINT. One variable, one owner per transition.
 *
 * It replaces two flags and, more importantly, the use of "xferrscidx ==
 * INVALID" as a proxy for "no transfer is running". The proxy could not tell
 * "we do not know the index yet" from "there is no transfer", so a lost Command
 * Complete left the endpoint reading as resourceless for ever and the recovery
 * answered by starting ANOTHER transfer - each taking a fresh transfer resource
 * and never returning it, until the controller ran out.
 *
 * INVARIANTS, checkable from udc_dwc3_ep_state_set() and its callers:
 *   1. DEPSTRTXFER is issued ONLY from IDLE.
 *   2. DEPUPDXFER is issued ONLY from RUNNING.
 *   3. DEPENDXFER is issued ONLY from RUNNING - it needs the index, which is
 *      what RUNNING means.
 *   4. DEPSTARTCFG / DEPCFG / DEPXFERCFG belong to endpoint enable and run only
 *      from IDLE; re-issuing DEPXFERCFG allocates another transfer resource, so
 *      it must never appear on a recovery path.
 *   5. Each transition is written next to the command that causes it.
 *
 * cfg.stat.enabled and cfg.stat.halted are NOT folded in: udc_common.c owns
 * them, and a second copy is the class of bug this enum removes.
 */
enum udc_dwc3_ep_state {
	UDC_DWC3_EP_IDLE = 0,	  /* no transfer, no controller resource held   */
	UDC_DWC3_EP_STARTING,	  /* DEPSTRTXFER issued, outcome not yet known  */
	UDC_DWC3_EP_RUNNING,	  /* transfer live, xferrscidx valid            */
	UDC_DWC3_EP_ENDING,	  /* DEPENDXFER issued, awaiting completion     */
	UDC_DWC3_EP_ENDING_RESUME,/* as ENDING, with a resume queued behind it  */
};

/* Why the machine left IDLE - carried so the report names the evidence. */
enum udc_dwc3_recov_reason {
	UDC_DWC3_RECOV_R_NONE = 0,
	UDC_DWC3_RECOV_R_UNPOPPED,	/* O2 HWO=0, armed_len != 0                */
	UDC_DWC3_RECOV_R_UNFETCHED,	/* O3 HWO=1, nothing retired               */
	UDC_DWC3_RECOV_R_NO_RESOURCE,	/* O4 xferrscidx INVALID with a live ring  */
	UDC_DWC3_RECOV_R_ENDXFER_LOST,	/* O5 stuck in ENDING                      */
	UDC_DWC3_RECOV_R_NO_SETUP,	/* O6 EP0 unarmed while the host asks      */
	UDC_DWC3_RECOV_R_SLOT_LOST,	/* O1 event announced, never written       */
};

struct udc_dwc3_recov {
	uint8_t  state;			/* enum udc_dwc3_recov_state */
	uint8_t  reason;		/* enum udc_dwc3_recov_reason */
	uint8_t  attempts;		/* actions taken in THIS episode */
	/*
	 * Episodes that were declared FAILED and whose endpoint then retired
	 * anyway. n_failed alone conflates "recovery gave up" with "the endpoint
	 * stayed broken"; a retire is durable evidence it is moving again, so
	 * the two are recorded separately. uint8_t saturating: this is a
	 * diagnostic, and it fits the padding after attempts at no RAM cost.
	 */
	uint8_t  n_failed_retired;
	uint32_t t0;			/* cycle stamp this state was entered */
	uint32_t n_recovered;		/* episodes that ended in a retire */
	uint32_t n_failed;		/* episodes that ended in FAILED */
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
	/* Record of the descriptor most recently armed on THIS endpoint. */
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
	/*
	 * UDC_DWC3_XFERRSCIDX_INVALID until an index has been recorded, which is
	 * also how the two collection paths know there is something to collect:
	 */
	/*
	 * Set when this endpoint's armed transfer has been invalidated but its
	 * completion event is still queued behind us.
	 */
	/*
	 * Set when an End Transfer was issued with CmdIOC, cleared by the
	 * resulting Endpoint Command Complete event.
	 */
	/*
	 * What this endpoint is doing, as one value. See enum udc_dwc3_ep_state
	 * for the invariants;
	 */
	uint8_t xfer_state;
	/* THE recovery state for this endpoint - see udc_dwc3_recover_step(). */
	struct udc_dwc3_recov recov;
	/* n_retire as of the last recovery evaluation - forward-progress detector. */
	uint32_t recov_retire_mark;
	/*
	 * priv->recov_discard_seq as of the last evaluation. The drain thread
	 * only ever increments that counter;
	 */
	uint32_t recov_discard_mark;
	/*
	 * Whether the postponed resume meant Init or Modify. Captured when the
	 * resume defers (the endpoint moves to ENDING_RESUME), because by the time
	 * it runs the stack has already set cfg.stat.enabled and the answer can no
	 * longer be read off it.
	 */
	/*
	 * Arms and retires on this endpoint. The usbmon capture of A capture
	 * showed EP01 bulk OUT stop accepting host writes 465 s BEFORE control
	 * wedged, with nothing logged device-side at all - the only trace was
	 * the SRP loop slowing down, which had been dismissed as a flaky test.
	 */
	uint32_t n_arm;
	uint32_t n_retire;
	/* Update Transfer commands refused while a descriptor was already armed. */
	uint32_t n_update_refused;
	/*
	 * Start Transfers that failed with a status and were retried from the
	 * Command Complete handler.
	 */
	/*
	 * Length programmed into each ring slot at arm time. The hardware rewrites
	 * BUFSIZ to the bytes NOT sent, so "armed 12, reads 0" is a writeback and
	 * "armed 0, reads 0" never carried data - the two readings of a TRB found
	 * with HWO set and BUFSIZ zero.
	 */
	uint32_t armed_len[CONFIG_UDC_DWC3_TRB_NUM];
	/*
	 * Whether the postponed resume meant Init or Modify. Captured when the
	 * resume defers, because by the time it runs the stack has already set
	 * cfg.stat.enabled and the answer can no longer be read off it.
	 */
	bool resume_modify;
	/*
	 * The transfer-resource pool generation this endpoint last ran DEPXFERCFG
	 * for. DEPXFERCFG ALLOCATES resources; re-issuing it without an
	 * intervening DEPSTARTCFG allocates more, and nothing but DEPSTARTCFG
	 * gives them back. Comparing this against priv->xfercfg_epoch is what
	 * makes the allocation once per endpoint per generation.
	 */
	uint32_t xfercfg_epoch;
};

/*
 * Data of each instance of the driver, that can be read and written to.
 *
 * Accessed via "udc_get_private(dev)".
 */
/*
 * What the event drain is doing right now.
 */
enum udc_dwc3_drain_state {
	UDC_DWC3_DRAIN_IDLE = 0,	/* controller owes nothing */
	UDC_DWC3_DRAIN_RUNNING,		/* taking events */
	UDC_DWC3_DRAIN_WAITING,		/* head slot empty, inside the budget */
	/*
	 * Head slot still empty past the arrival budget. A NUDGE is a DGCMD
	 * generic command issued purely to make the controller write an event,
	 * which unblocks a ring the controller owes events for but has not
	 * written. Sent by udc_dwc3_nudge_worker().
	 */
	UDC_DWC3_DRAIN_NUDGE,
	UDC_DWC3_DRAIN_PARTIAL,		/* pass ended on a mid-pass empty slot */
};

/*
 * Drain-owned state. Every field is a 32-bit word with exactly one writer, so a
 * reader on another thread can never see a half-written value - do NOT widen any
 * of these to 64 bits, that guarantee is what makes the lockless read legal.
 */
struct udc_dwc3_drain {
	uint32_t state;		/* enum udc_dwc3_drain_state */
	uint32_t slot;		/* slot the episode is stuck on */
	uint32_t since;		/* cycle stamp the episode opened */
	uint32_t attempts;	/* consecutive give-ups on that slot */
	uint32_t gc0;		/* GEVNTCOUNT when the episode opened */
	uint32_t quiet;		/* nothing was printed inside this episode */
	uint32_t counted;	/* this episode was already counted as missed */

};

/* Reset the whole drain state as one act - see the enum above. */
static inline void udc_dwc3_drain_reset(struct udc_dwc3_drain *const d)
{
	*d = (struct udc_dwc3_drain){ .state = UDC_DWC3_DRAIN_IDLE };
}

struct udc_dwc3_data {
	DEVICE_MMIO_NAMED_RAM(base);
	/* Index within trb where to queue new TRBs */
	uint32_t evt_next;
	/* Back-reference to parent */
	const struct device *dev;
	/* Dispatch from IRQ events to workqueue jobs */
	struct k_sem evt_sem;		/* ISR -> drain thread */
	k_thread_stack_t *evt_stack;
	struct k_thread *evt_thread;
	uint32_t evt_stack_free;	/* smallest observed headroom, bytes */
	/* A work queue entry to test if the previous transaction is stuck */
	struct k_work_delayable watchdog_dwork;
	/* First endpoint to be configured */
	uint32_t ep_cmd_cmplt_stale;	/* End Transfer completion for a dead incarnation */
	uint8_t first_ep;
	/*
	 * DEPSTARTCFG has been issued for this configuration. It reassigns the
	 * controller's transfer-resource pool AND resets every endpoint's state,
	 * so re-running it on each SET_INTERFACE that happens to touch first_ep
	 * tears down endpoints that are streaming. Cleared on bus reset, which
	 * is where a genuinely new configuration begins.
	 */
	bool cfg_pool_assigned;
	/*
	 * Transfer-resource pool generation. Bumped by every DEPSTARTCFG, which
	 * is the only thing that reassigns the controller's pool.
	 */
	uint32_t xfercfg_epoch;
#if CONFIG_UDC_DWC3_SHELL
	/* FIFO space initial values */
	uint16_t max_bytes_avail[_NUM_FIFO_SPACE][_NUM_FIFO_REGS];
#endif
	/* Next expected control transfer */
	/* Updated whenever a packet is submitted */
	uint32_t last_xfer_type;
	uint8_t last_xfer_dir;
	/* Cached TRBs to recover from corrputed TRBs */
	/* Cache that is always up to date (before stack could get time to react) */
	struct usb_setup_packet setup_packet;
	/*
	 * Diagnostics for the event-buffer posted-write race. evt_late counts
	 * the events whose write had not landed on the first read;
	 */
	uint32_t evt_late;
	uint32_t evt_gaveup;
	uint32_t evt_handled;
	/*
	 * Instrumentation for telling apart the two remaining explanations of a
	 * stall:
	 */
	/*
	 * The pass's ONE read of GEVNTCOUNT, published for every other reader.
	 * It is a decrement-on-write counter the controller updates
	 * concurrently, so two reads never describe the same instant. Read once
	 * at the top of udc_dwc3_evt_drain() and once in udc_dwc3_drain_helper();
	 * everything else uses this copy.
	 */
	uint32_t evt_gc_last;
	uint32_t evt_gevntcount_hwm;	/* worst announced-but-unread backlog, bytes */
	uint32_t evt_sweep_rescued;	/* completions the heartbeat sweep returned */
	uint32_t evt_sweep_runs;	/* sweeps that found something to drain */
	uint32_t evt_lookahead_short;	/* stalls ended early by the look-ahead proof */
	bool ctrl_stall_captured;	/* a CTRLSTALL capture is waiting for its pair */
	struct udc_dwc3_drain drain;	/* THE drain state - see the enum above */
	uint32_t evt_gaveup_us_max;	/* worst UNINSTRUMENTED fill latency, us */
	uint32_t evt_zero;		/* slots the CONTROLLER wrote as 0x00000000 */
	uint32_t evt_missed;		/* give-up runs presumed a LOST write */
	uint32_t evt_missed_frozen;	/* of those, with GEVNTCOUNT not moving */
	/*
	 * Control handshake trace. Every control transfer with a data stage runs
	 * SETUP-completes -> reported up -> stack enqueues the data buffer ->
	 * ctrl_try arms it.
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
	uint32_t evt_force_t0;		/* cycle stamp of the last forced command */
	uint32_t evt_copy[CONFIG_UDC_DWC3_EVENTS_NUM];
	uint32_t evt_worker_exit_t0;	/* cycle stamp when the worker last EXITED */
	uint32_t evt_kick;		/* heartbeat had to restart a stopped drain */
	/*
	 * Interrupts taken versus worker passes entered. These two answer a
	 * question the rest of the instrumentation cannot:
	 */
	uint32_t evt_isr;		/* interrupt handler invocations */
	uint32_t evt_worker_runs;	/* event worker passes entered */
	uint32_t evt_skipped;		/* events discarded to free a full ring */
	uint32_t evt_link_total;	/* USB/Link State Change events seen */
	uint32_t evt_link_run;		/* consecutive events reporting the same state */
	uint32_t evt_link_last;		/* that state, EvtInfo[3:0] */
	/* Liveness, watched from the system work queue - see the heartbeat worker. */
	/*
	 * The heartbeat is driven by a PERIODIC KERNEL TIMER, not by a delayed
	 * work item that re-arms itself.
	 */
	struct k_timer heartbeat_timer;
	struct k_work heartbeat_work;
	/* Runs udc_dwc3_evt_force(): one DGCMD to make the controller emit. */
	struct k_work nudge_work;
	/*
	 * Serialises the DGCMDPAR/DGCMD pair, whose two writers are
	 * udc_dwc3_fifo_flush_tx() on the drain thread and udc_dwc3_evt_force()
	 * on udc_get_work_q().
	 */
	struct k_spinlock dgcmd_lock;
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
	 * cleared before it releases.
	 */
	/* Where the event ring is copied to before it is acknowledged. */
	/* Set when a drain pass stopped on an empty slot rather than finishing. */
	uint32_t evt_midzero;		/* how many passes ended that way */
	/* The stuck-control-claim detector. */
	uint32_t ctrl_decline;		/* ctrl_try() turned the stack away */
	uint32_t ctrl_recover;		/* stuck claims broken by the heartbeat */
	uint32_t ctrl_arm_t0;		/* cycle stamp of the last granted claim */
	bool ctrl_decline_pending;	/* declined since the last grant */
	uint32_t ctrl_decline_t;	/* cycle stamp of the most recent decline */
	uint32_t hb_last_setup_done;	/* ctrl_setup_done at the previous beat */
	/* Beats in a row with events pending in the ring and nothing consumed. */
	uint32_t hb_last_handled;
	uint32_t hb_drain_stuck_beats;
	uint32_t core_dbg_beats;	/* beats since the last CORE debug sample */
	uint32_t ctrl_quiet_t0;		/* cycle stamp of the last SETUP retired */
	bool ctrl_quiet_logged;		/* this quiet period already reported */
	uint32_t ctrl_start_fail;	/* Start Transfer commands rejected */
	/*
	 * NEVER SET - see udc_dwc3_recover(). This was meant to mean "recover()
	 * has issued an End Transfer on a control endpoint and is waiting for
	 * its Endpoint Command Complete event", but recover() issues Set Stall
	 * and no End Transfer, so nothing assigns this true and every path
	 * guarded by it is dead.
	 */
	/*
	 * The endpoint recovery ended, remembered so the re-arm cannot follow
	 * last_xfer_dir somewhere else.
	 */
	/* The control endpoint and stage the watchdog is guarding. */
	struct udc_dwc3_ep_data *watchdog_ep;
	uint32_t watchdog_type;
	/*
	 * How many control transfers the host abandoned by starting a new SETUP,
	 * and how many completions carried some other non-OK TRBSTS.
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
	 * Where this control transfer has got to. One value, not a set of flags.
	 * a request that ended without a status stage left the "data stage done"
	 * flag set, the next request inherited it, and its first
	 * XferNotReady(Data) was judged against the previous transfer - reported
	 * as 4.4.2 step 5b, "host is moving more data than wLength", against a
	 * host that had asked for nine bytes of configuration descriptor.
	 */
	enum udc_dwc3_ctrl_state {
		UDC_DWC3_CTRL_IDLE = 0,
		UDC_DWC3_CTRL_SETUP_DONE,
		UDC_DWC3_CTRL_DATA_DONE,
		UDC_DWC3_CTRL_STATUS_READY,
		UDC_DWC3_CTRL_STATUS_ARMED,
	} ctrl_state;
	/*
	 * The last transition, kept so a wedge dump can say how the machine got
	 * to the state it is stuck in.
	 */
	uint8_t  ctrl_state_prev;	/* state we came from */
	uint16_t ctrl_state_seq;	/* transitions since boot, wraps */
	uint32_t ctrl_status_defer;	/* status arms held back waiting for it */
	uint32_t ctrl_desync;		/* spec error cases caught and recovered */
	uint32_t ctrl_stall_issued;	/* Set Stalls issued for steps 2 and 5b */
	uint32_t ep_halts;		/* halts set on non-control endpoints */
	/* The two ends of a control transfer, counted independently. */
	uint32_t ctrl_setup_done;	/* SETUP stages retired */
	/*
	 * Retires on every non-control endpoint, used purely as a liveness proxy
	 * by the SETUP watchdog:
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
	 * Update Transfer issued at a stuck SETUP to force the controller to re-
	 * cache the descriptor (databook 3.2.2.6).
	 */
	uint32_t ctrl_setup_wd_updxfer;
	uint32_t ctrl_recover_mark;	/* stage total at the last non-SETUP Set Stall */
	uint32_t ctrl_wd_dump;		/* non-SETUP watchdog fires dumped so far */
	uint32_t ctrl_wd_upd_mark;	/* stage count at the last control re-cache */
	bool buscfg_logged;		/* the one-shot bus/DMA config report has run */
	/* Descriptors overwritten while the controller still owned them. */
	uint32_t trb_stomp;
	uint32_t ctrl_arm_refused;
	uint32_t nonctrl_recache;	/* re-caches issued on non-control endpoints */
	uint32_t ctrl_setup_wd_upd_mark;
	uint32_t ctrl_status_done;	/* status stages retired (IN and OUT) */
	uint32_t ctrl_trbsts_other;
	/*
	 * How often udc_dwc3_ctrl_try() declined to arm because the endpoint's
	 * End Transfer had not reported completion.
	 */
	uint32_t ctrl_deferred_arm;
	/*
	 * One bit per physical endpoint, set the first time this driver writes
	 * that endpoint's DEPCMD register. Databook section 1.3.12, DEPCMD[0:7]:
	 */
	uint32_t depcmd_issued;
	/*
	 * Command accounting. depcmd_issued above is a per-endpoint BITMAP of
	 * "has this endpoint ever been commanded"; these are counts.
	 */
	uint32_t depcmd_n_ok;		/* completion seen, CmdStatus OK        */
	uint32_t depcmd_n_err;		/* completion seen, CmdStatus != OK     */
	uint32_t depcmd_n_timeout;	/* fast poll expired, status unknown    */
	uint32_t depcmd_n_late_ok;	/* retired after the poll, before the log */
	uint32_t depcmd_n_notissued;	/* pre-poll found the previous still active */
	uint32_t recov_discard_seq;	/* events the drain threw away (drain-written) */
	/* Heartbeat liveness, measured rather than inferred. */
	uint32_t hb_beats;
	uint32_t hb_gap_ms_max;
	uint32_t hb_last_t;
	/* Queue latency, to tell two different causes of a late beat apart. */
	uint32_t hb_submit_t;
	uint32_t hb_q_ms_max;
	/*
	 * The timer does not drift. k_timer_start() armed it periodic, so every
	 * expiry lands on the UDC_DWC3_HEARTBEAT_MS grid whatever the load - a
	 * periodic k_timer re-arms from the expiry point, not from when the
	 * callback got around to running. What can be lost is the WORK RUN.
	 */
	uint32_t hb_expiries;
	uint32_t hb_coalesced;
	uint32_t evt_ack_dead;		/* consecutive acks that GEVNTCOUNT ignored */
	/*
	 * THE ONE OUTSTANDING EVENT-RING CREDIT. Writing GEVNTCOUNT is a POSTED
	 * MMIO write: the read-back that follows it can still show the old value
	 * while the write is in flight, so "unchanged" does not mean "refused".
	 * Without this record the next give-up simply issued the credit again,
	 * and if both eventually landed the controller was credited twice for
	 * ONE skipped event - it would then believe a slot was free that
	 * software has never read, and overwrite it.
	 */
	bool evt_credit_pending;
	uint32_t evt_credit_slot;	/* evt_next when the credit was issued   */
	uint32_t evt_credit_skip;	/* slots it credited                     */
	uint32_t evt_credit_gc;		/* GEVNTCOUNT observed just before it    */
	/* Last command word posted per physical endpoint, and a bit per endpoint
	 * saying its failure has already been reported at issue time. DEPCMD keeps
	 * the command type but not the parameters we passed, and the one-command-late
	 * report would otherwise print the same failure a second time.
	 */
	uint32_t depcmd_last[UDC_DWC3_MAX_EPN];
	uint32_t depcmd_reported;
	/*
	 * One bit per physical endpoint, set once a failed Start Transfer has
	 * been announced on that endpoint and cleared only when the next Start
	 * Transfer is posted.
	 */
	uint32_t start_fail_reported;
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
/*
 * Re-establish an endpoint: configure it, enable it in DALEPENA and arm whatever
 * is queued. modify selects DEPCFG Modify over Init.
 */
static int udc_dwc3_ep_resume(const struct device *const dev,
			      struct udc_dwc3_ep_data *const ep_data,
			      const bool modify);
/* Defined next to the control stage checks that are its other caller. */
static void udc_dwc3_fifo_flush_tx(const struct device *const dev, const uint8_t fifo);

/*
 * A control request is finished with: its status stage has been armed, so the
 * only thing still owed on it is a completion, never another XferNotReady.
 */
/*
 * THE ONLY PLACE ctrl_state IS ASSIGNED.
 */
static void udc_dwc3_ctrl_state_set(const struct device *const dev,
				    const enum udc_dwc3_ctrl_state next)
{
	struct udc_dwc3_data *const priv = udc_get_private(dev);

	priv->ctrl_state_prev = (uint8_t)priv->ctrl_state;
	priv->ctrl_state_seq++;
	priv->ctrl_state = next;
}

#ifdef CONFIG_UDC_DWC3_SHELL
static void udc_dwc3_init_fifo_space(const struct device *dev);
#endif

#ifdef CONFIG_UDC_DWC3_SHELL
/*
 * Log one TRB's fields.
 */
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

/*
 * UDC API lock/unlock. Thin wrappers over the framework mutex.
 */
static void udc_dwc3_lock(const struct device *const dev)
{
	udc_lock_internal(dev, K_FOREVER);
}

/*
 * See udc_dwc3_lock().
 */
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
 */
/* Ceiling on the CSftRst completion wait - see udc_dwc3_on_soft_reset(). */
/*
 * Bounded CSR reads first, then sleeping ticks. No busy-wait: this is reachable
 * at runtime from the stuck-SETUP escalation, on the work queue under the mutex.
 */
#define UDC_DWC3_CSFTRST_FAST_READS 256u
#define UDC_DWC3_CSFTRST_SLOW_TICKS 10u
#define UDC_DWC3_CMD_FAST_POLLS 32u
#define UDC_DWC3_CMD_FAST_POLL_US 1u
/*
 * Bare reads before the loop starts yielding. Most endpoint commands retire in
 * under a microsecond; a reschedule for those costs more than it saves.
 */
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

/* Defined with the other event-name decoders; used here for timeout diagnostics. */
static const char *udc_dwc3_get_devt_ulstchng_name(const uint32_t dsts);

/*
 * Physical endpoint number behind a DEPCMD register address, for the
 * priv->depcmd_issued bookkeeping. The addresses all come from
 * UDC_DWC3_DEPCMD(n), so this simply undoes that macro.
 */
/*
 * F11: the inverse of UDC_DWC3_DEPCMD(n), bounds-checked.
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
 * Poll DEPCMD.CmdAct until the controller clears it. Returns false if the
 * command is still active when the bounded budget expires; *reg_out is the last
 * value read.
 */
static bool udc_dwc3_wait_cmdact_zero(const struct device *const dev,
				      const uint32_t addr, uint32_t *const reg_out)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint32_t reg = 0;

	/*
	 * A BOUNDED READ LOOP, WITH NO YIELD.
	 *
	 * k_yield() cannot do here what it looks like it does. On the drain
	 * thread - K_PRIO_COOP(6), the highest priority in the system - nothing
	 * else is ready at that level, so it returns immediately and the loop
	 * spins anyway with the UDC mutex held. On the work-queue thread it does
	 * the opposite and worse: it hands the CPU to the drain thread WHILE
	 * THIS MUTEX IS HELD, and the drain can sit in its arrival wait for its
	 * whole budget before it blocks on the mutex.
	 *
	 * The loop is a few dozen CSR reads, and every caller already handles a
	 * false return, so giving up is cheap and safe.
	 */
	for (uint32_t i = 0; i < UDC_DWC3_CMD_FAST_POLLS; i++) {
		reg = sys_read32(base + addr);
		if ((reg & UDC_DWC3_DEPCMD_CMDACT) == 0) {
			*reg_out = reg;
			return true;
		}
	}

	/*
	 * THAT IS THE WHOLE WAIT. IT DOES NOT SLEEP, EVER. Every DEPCMD in this
	 * driver is issued with the UDC mutex held, and udc_dwc3_handle_event()
	 * takes that same mutex for every event - so any sleep here parks the
	 * event drain and the recovery machinery behind it, for the whole sleep,
	 * on a core that has stopped answering.
	 */
	if (reg_out != NULL) {
		*reg_out = reg;
	}

	return false;
}

/*
 * Issue an endpoint command.
 */


/*
 * Make a descriptor visible to the controller before the command that fetches it.
 */
static inline void udc_dwc3_trb_sync(volatile uint32_t *const last_word)
{
	/* The read-back that used to be here is deliberately gone. */
	/* uint32_t readback = *last_word; */
	/* (void)readback; */
	ARG_UNUSED(last_word);

#if defined(CONFIG_RISCV)
	__asm__ volatile ("fence iorw,iorw" ::: "memory");
#else
	barrier_dsync_fence_full();
#endif
}

/* Bounded: the first few are what matter, and this may fire in a tight loop. */
#define UDC_DWC3_TRB_STOMP_LOG_FIRST				8u

/*
 * F6: one controller, asserted rather than assumed.
 */
BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) <= 1,
	     "udc_dwc3 is single-instance: udc_dwc3_stomp_priv is file-scope and "
	     "the last initialised controller would own every stomp report");

static struct udc_dwc3_data *udc_dwc3_stomp_priv;

/*
 * Report a TRB that was overwritten while the controller still owned it.
 */
static inline void udc_dwc3_trb_stomp_report(volatile struct udc_dwc3_trb *const trb)
{
	struct udc_dwc3_data *const priv = udc_dwc3_stomp_priv;

	if (priv == NULL) {
		return;
	}

	priv->trb_stomp++;

	if (priv->trb_stomp <= UDC_DWC3_TRB_STOMP_LOG_FIRST) {
		LOG_ERR("overwriting a descriptor the controller still owns: "
			"ctrl 0x%08x sts 0x%08x (stomp %u)",
			trb->ctrl, trb->status, priv->trb_stomp);
	}
}

/*
 * Fill a TRB and make it visible to the controller. The ONLY place a TRB's
 * words are written.
 */
static inline void udc_dwc3_trb_fill(volatile struct udc_dwc3_trb *const trb,
				     const uintptr_t addr, const uint32_t status,
				     const uint32_t ctrl)
{
	/*
	 * Report - do not prevent - overwriting a descriptor the controller
	 * still owns.
	 */
	if ((trb->ctrl & UDC_DWC3_TRB_CTRL_HWO) != 0U) {
		udc_dwc3_trb_stomp_report(trb);
	}

	trb->addr_lo = LO32(addr);
	trb->addr_hi = HI32(addr);
	trb->status = status;
	trb->ctrl = ctrl;

	udc_dwc3_trb_sync(&trb->ctrl);
}

/*
 * Record a transfer resource index. See the definition below.
 */
static void udc_dwc3_store_xferrscidx(const struct device *const dev,
				      struct udc_dwc3_ep_data *const ep_data,
				      uint32_t idx);

/*
 * Name of an enum udc_dwc3_ep_state value, for logging.
 */
static const char *udc_dwc3_ep_state_name(const uint8_t st)
{
	switch (st) {
	case UDC_DWC3_EP_IDLE:		return "idle";
	case UDC_DWC3_EP_STARTING:	return "starting";
	case UDC_DWC3_EP_RUNNING:	return "running";
	case UDC_DWC3_EP_ENDING:	return "ending";
	case UDC_DWC3_EP_ENDING_RESUME:	return "ending+resume";
	default:			return "?";
	}
}

/*
 * The ONLY writer of xfer_state.
 */
static bool udc_dwc3_ep_state_set(struct udc_dwc3_ep_data *const ep_data,
				  const uint8_t next)
{
	static const uint8_t legal[][2] = {
		{ UDC_DWC3_EP_IDLE,		UDC_DWC3_EP_STARTING },
		/*
		 * Control endpoints only: each stage is armed with its own Start
		 * Transfer, so EP0 goes RUNNING -> STARTING at every stage
		 * boundary without an End Transfer between them.
		 */
		{ UDC_DWC3_EP_RUNNING,		UDC_DWC3_EP_STARTING },
		{ UDC_DWC3_EP_STARTING,		UDC_DWC3_EP_RUNNING },
		{ UDC_DWC3_EP_STARTING,		UDC_DWC3_EP_IDLE },
		{ UDC_DWC3_EP_RUNNING,		UDC_DWC3_EP_ENDING },
		/*
		 * The End Transfer was never issued - the pre-poll found the
		 * previous command on this endpoint still active and gave up.
		 */
		{ UDC_DWC3_EP_ENDING,		UDC_DWC3_EP_RUNNING },
		{ UDC_DWC3_EP_RUNNING,		UDC_DWC3_EP_IDLE },
		{ UDC_DWC3_EP_ENDING,		UDC_DWC3_EP_ENDING_RESUME },
		{ UDC_DWC3_EP_ENDING,		UDC_DWC3_EP_IDLE },
		{ UDC_DWC3_EP_ENDING_RESUME,	UDC_DWC3_EP_IDLE },
		/*
		 * Teardown withdraws a queued resume. The End Transfer is still
		 * outstanding, so the endpoint stays ENDING; only the intent to
		 * re-arm afterwards is dropped.
		 */
		{ UDC_DWC3_EP_ENDING_RESUME,	UDC_DWC3_EP_ENDING },
	};

	if (ep_data->xfer_state == next) {
		return true;
	}

	for (size_t i = 0; i < ARRAY_SIZE(legal); i++) {
		if (legal[i][0] == ep_data->xfer_state && legal[i][1] == next) {
			LOG_DBG("EP%02x xfer %s -> %s", ep_data->cfg.addr,
				udc_dwc3_ep_state_name(ep_data->xfer_state),
				udc_dwc3_ep_state_name(next));
			ep_data->xfer_state = next;
			return true;
		}
	}

	LOG_ERR("EP%02x ILLEGAL transfer state change %s -> %s, refused",
		ep_data->cfg.addr, udc_dwc3_ep_state_name(ep_data->xfer_state),
		udc_dwc3_ep_state_name(next));

	return false;
}

/* An End Transfer is outstanding, with or without a resume queued behind it. */
static inline bool udc_dwc3_ep_is_ending(const struct udc_dwc3_ep_data *const ep_data)
{
	return ep_data->xfer_state == UDC_DWC3_EP_ENDING ||
	       ep_data->xfer_state == UDC_DWC3_EP_ENDING_RESUME;
}

/*
 * Back to IDLE from anywhere, for the paths that are entitled to do it: bus
 * reset, disconnect, endpoint disable and controller recovery. These do not
 * "transition" so much as declare that whatever the controller was doing is
 * over, so they bypass the table rather than being listed as eight more rows.
 */
static void udc_dwc3_ep_state_reset(struct udc_dwc3_ep_data *const ep_data)
{
	if (ep_data->xfer_state != UDC_DWC3_EP_IDLE) {
		LOG_DBG("EP%02x xfer %s -> idle (reset)", ep_data->cfg.addr,
			udc_dwc3_ep_state_name(ep_data->xfer_state));
	}

	ep_data->xfer_state = UDC_DWC3_EP_IDLE;
	ep_data->xferrscidx = UDC_DWC3_XFERRSCIDX_INVALID;
}

/*
 * Adopt a transfer resource index out of DEPCMD. See the definition below.
 */
static void udc_dwc3_adopt_xferrscidx(const struct device *const dev,
				      struct udc_dwc3_ep_data *const ep_data,
				      const uint32_t reg);

/*
 * Issue one endpoint command and collect its result.
 *
 * Returns 0 when the command completed with status OK, and
 * UDC_DWC3_XFERRSCIDX_INVALID when it was rejected OR was still executing when
 * the poll expired - callers that must tell those apart re-read DEPCMD.CmdAct.
 */
static uint32_t udc_dwc3_depcmd(const struct device *const dev,
				const uint32_t addr, const uint32_t cmd)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const uint32_t epn = udc_dwc3_depcmd_epn(addr);
	const bool first_on_ep = (epn >= UDC_DWC3_MAX_EPN) ||
				 ((priv->depcmd_issued & BIT(epn)) == 0);
	/*
	 * Nothing is waited on for its result any more. Start Transfer used to be,
	 * which cost up to UDC_DWC3_CMD_FAST_POLLS reads of the shared CSR port for
	 * every armed transfer - once per buffer on the bulk path.
	 */
	uint32_t reg = 0;

	/*
	 * A command must not be issued while the previous one on this endpoint
	 * is still active. Databook section 1.3.12 says the read value of DEPCMD
	 * is undefined until the first endpoint command is issued on it, that
	 * CmdAct in particular may come up set, and that issuing a command
	 * anyway is safe.
	 */
	/*
	 * Do not write a command over one that is still active. Every command is
	 * post-polled below, so in the normal case CmdAct is already clear and
	 * this costs one CSR read. Writing the next command then lands on an
	 * active one - DEPCMD CmdAct is R/W1S and the databook does not define
	 * the result, so the command may be dropped, doubled, or applied to the
	 * previous command's parameters.
	 */
	if (!first_on_ep && !udc_dwc3_wait_cmdact_zero(dev, addr, &reg)) {
		LOG_ERR("previous command still active on addr 0x%x (0x%08x) after the "
			"bounded poll, not issuing command 0x%x, GEVNTCOUNT=%u bytes, "
			"DSTS=0x%08x (%s)",
			addr, reg, cmd,
			priv->evt_gc_last,
			sys_read32(base + UDC_DWC3_DSTS),
			udc_dwc3_get_devt_ulstchng_name(sys_read32(base + UDC_DWC3_DSTS)));
		priv->depcmd_n_notissued++;
		return UDC_DWC3_XFERRSCIDX_INVALID;
	}

	if (!first_on_ep) {
		const struct udc_dwc3_config *const cfg = DEV_CFG(dev);

		if (_EPN_IS_VALID(cfg, epn)) {
			udc_dwc3_adopt_xferrscidx(dev, _EP_DATA_FROM_EPN(cfg, epn), reg);
		}
	}

	if (!first_on_ep &&
	    (reg & UDC_DWC3_DEPCMD_STATUS_MASK) == UDC_DWC3_DEPCMD_STATUS_CMDERR &&
	    (epn >= UDC_DWC3_MAX_EPN || (priv->depcmd_reported & BIT(epn)) == 0)) {
		LOG_ERR("previous endpoint command on addr 0x%x reported an error "
			"(0x%08x): command 0x%08x, type 0x%x", addr, reg,
			epn < UDC_DWC3_MAX_EPN ? priv->depcmd_last[epn] : 0u,
			epn < UDC_DWC3_MAX_EPN ? (unsigned int)(priv->depcmd_last[epn] &
						   UDC_DWC3_DEPCMD_CMDTYP_MASK) : 0u);
	}

	reg = sys_read32(base + UDC_DWC3_GUSB2PHYCFG);
	if ((reg & (UDC_DWC3_GUSB2PHYCFG_SUSPHY |
		    UDC_DWC3_GUSB2PHYCFG_ENBLSLPM)) != 0) {
		/*
		 * Clear them and LEAVE them clear. Restoring afterwards would mean
		 * waiting for the command to finish first - the databook requires the
		 * bits stay clear for its whole execution, not just at issue - and
		 * that wait is a SECOND poll of CmdAct in the same call, on top of the
		 * pre-poll above. One poll per command is the rule here.
		 */
		sys_write32(reg & ~(UDC_DWC3_GUSB2PHYCFG_SUSPHY |
				    UDC_DWC3_GUSB2PHYCFG_ENBLSLPM),
			    base + UDC_DWC3_GUSB2PHYCFG);

		LOG_WRN("GUSB2PHYCFG had SUSPHY/ENBLSLPM set (0x%08x) before an "
			"endpoint command; cleared and left clear", reg);
	}

	/*
	 * Every endpoint command on every endpoint - control, bulk and interrupt
	 * alike - is issued from here, so this is the one place that can promise
	 * the ordering the controller depends on:
	 */
	/*
	 * The transfer resource index of this endpoint lives and dies with the
	 * two commands that bracket a transfer, so both are handled here rather
	 * than at the call sites. Programming Guide 3.2.2.2:
	 */
	{
		const struct udc_dwc3_config *const cfg = DEV_CFG(dev);
		const uint32_t cmdtyp = cmd & UDC_DWC3_DEPCMD_CMDTYP_MASK;

		if (_EPN_IS_VALID(cfg, epn) &&
		    (cmdtyp == UDC_DWC3_DEPCMD_DEPSTRTXFER ||
		     cmdtyp == UDC_DWC3_DEPCMD_DEPENDXFER)) {
			_EP_DATA_FROM_EPN(cfg, epn)->xferrscidx =
				UDC_DWC3_XFERRSCIDX_INVALID;
		}
	}

	sys_write32(cmd | UDC_DWC3_DEPCMD_CMDACT, base + addr);

	/*
	 * From here the register has been written, so its read value is defined and
	 * the pre-poll above may use it on the next command for this endpoint.
	 */
	if (epn < UDC_DWC3_MAX_EPN) {
		priv->depcmd_issued |= BIT(epn);
		priv->depcmd_last[epn] = cmd;
		priv->depcmd_reported &= ~BIT(epn);

		/*
		 * A new Start Transfer begins a new transfer, so any failure
		 * reported for the previous one must not suppress the next one's
		 * failure.
		 */
		if ((cmd & UDC_DWC3_DEPCMD_CMDTYP_MASK) ==
		    UDC_DWC3_DEPCMD_DEPSTRTXFER) {
			priv->start_fail_reported &= ~BIT(epn);
		}
	}

	/*
	 * Start Transfer is the ONE command this driver waits for; SPEC, section
	 * 1.3.12, the DEPCMD CommandParam field:
	 */
	/*
	 * POST-POLL EVERY COMMAND. Only Start Transfer used to be waited for.
	 * Update Transfer carries CmdIOC=0, so it raises no Command Complete
	 * event, and nothing polled it - its CmdStatus was never read by
	 * anything, ever.
	 */
	if (epn < UDC_DWC3_MAX_EPN) {
		const struct udc_dwc3_config *const cfg = DEV_CFG(dev);
		const uint32_t cmdtyp = cmd & UDC_DWC3_DEPCMD_CMDTYP_MASK;
		uint32_t done = 0;
		bool finished;

		finished = udc_dwc3_wait_cmdact_zero(dev, addr, &done);

		/*
		 * LOOK ONCE MORE BEFORE REPORTING. The LOG_ERR below is ~130
		 * characters and CONFIG_LOG_MODE_MINIMAL makes every live log a
		 * synchronous uart_poll_out at ~87 us/char - about 11 ms with this
		 * mutex held. Callers answer a failed return by re-reading CmdAct
		 * to tell "still executing" from "rejected", and a command that
		 * retires during those 11 ms reads as CmdAct clear, so a SUCCESS
		 * is classified as a rejection: the endpoint is reset to IDLE and
		 * the transfer resource the controller just assigned is orphaned.
		 *
		 * One more register read costs nothing on a path that is already
		 * the rare one, and CmdStatus is valid the moment CmdAct clears.
		 */
		if (!finished) {
			done = sys_read32(base + addr);
			finished = (done & UDC_DWC3_DEPCMD_CMDACT) == 0U;
			if (finished) {
				priv->depcmd_n_late_ok++;
			}
		}

		if (!finished) {
			priv->depcmd_n_timeout++;
			/*
			 * UNKNOWN IS A FAILURE, and this used to return 0 - the
			 * same value a command that completed with status OK
			 * returns.
			 */
			LOG_ERR("EP%02x command 0x%x on addr 0x%x still active after "
				"the fast poll (0x%08x); status UNKNOWN, treating as "
				"failed (%u so far)",
				_EPN_IS_VALID(cfg, epn)
					? _EP_DATA_FROM_EPN(cfg, epn)->cfg.addr : 0xffU,
				cmd, addr, done, priv->depcmd_n_timeout);
			return UDC_DWC3_XFERRSCIDX_INVALID;
		}

		/*
		 * Start Transfer is the only command that assigns a transfer
		 * resource, and adopt applies its own three checks before trusting
		 * the value.
		 */
		if (cmdtyp == UDC_DWC3_DEPCMD_DEPSTRTXFER && _EPN_IS_VALID(cfg, epn)) {
			udc_dwc3_adopt_xferrscidx(dev, _EP_DATA_FROM_EPN(cfg, epn),
						  done);
		}

		if ((done & UDC_DWC3_DEPCMD_STATUS_MASK) !=
		    UDC_DWC3_DEPCMD_STATUS_OK) {
			priv->depcmd_n_err++;

			/*
			 * ERROR, unconditionally. This is the line that says a
			 * command the driver believed it had issued was refused by
			 * the controller - the thing nothing could see before.
			 */
			LOG_ERR("EP%02x command type 0x%x REJECTED: DEPCMD=0x%08x "
				"status=0x%x (%u errors so far)",
				_EPN_IS_VALID(cfg, epn)
					? _EP_DATA_FROM_EPN(cfg, epn)->cfg.addr : 0xffU,
				(unsigned int)(cmdtyp >> 0),
				done,
				(unsigned int)((done & UDC_DWC3_DEPCMD_STATUS_MASK) >> 12),
				priv->depcmd_n_err);

			priv->depcmd_reported |= BIT(epn);
			if (cmdtyp == UDC_DWC3_DEPCMD_DEPSTRTXFER) {
				priv->start_fail_reported |= BIT(epn);
			}

			return UDC_DWC3_XFERRSCIDX_INVALID;
		}

		priv->depcmd_n_ok++;
	}

	return 0;
}

/*
 * DEPCFG: program an endpoint's type, packet size, FIFO and interrupt number.
 */
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
	 * Init or Modify is passed in rather than inferred from
	 * cfg.stat.enabled.
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
	 * Burst Size is "number of packets per burst minus one". Control
	 * endpoints do not burst and Table 4-1 programs BrstSiz = 0 for them, so
	 * 0 is forced for endpoint 0.
	 */
	if (USB_EP_GET_IDX(ep_data->cfg.addr) == 0) {
		param0 |= FIELD_PREP(UDC_DWC3_DEPCMDPAR0_DEPCFG_BRSTSIZ_MASK, 0);
	} else {
		/* Burst of 4, matched to bMaxBurst=3 in the class descriptors and
		 * DCFG.NUMP=4 - all three aligned, as the controller vendor specified. */
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
	 * XferNotReady is mandatory on the control endpoints: "The XferNotReady
	 * event must not be disabled for control endpoints because the event is an
	 * integral part of control transfer handling" (section 4.2.4).
	 */
	/* XferNotReady on EP0 only. */
	/*
	 * EP0 only, and a non-control OUT endpoint genuinely does not need it.
	 * Control is the exception the databook names, and the reason is
	 * specific:
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
	 * "set to the bInterval value minus 1. When the controller is operating
	 * in Full-Speed mode, this field must be set to 0." Section 4.3.3 makes
	 * it mandatory for isochronous endpoints, and it carries the same
	 * meaning for interrupt ones.
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

/*
 * DEPXFERCFG: allocate this endpoint's transfer resources. Endpoint enable only -
 * re-issuing it allocates another resource that nothing returns.
 */
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

/*
 * Returns whether the controller actually accepted the command.
 */
static bool udc_dwc3_depcmd_set_stall(const struct device *const dev,
				      struct udc_dwc3_ep_data *const ep_data)
{
	LOG_DBG("DepSetStall: EP%02x", ep_data->cfg.addr);

	/*
	 * udc_dwc3_depcmd() reports 0 for success and UDC_DWC3_XFERRSCIDX_INVALID
	 * for failure - it does NOT return the DEPCMD register, so this must not
	 * be decoded as one.
	 */
	return udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn),
			      UDC_DWC3_DEPCMD_DEPSETSTALL) == 0U;
}

/*
 * DEPCSTALL. Returns whether the controller accepted the command.
 */
static bool udc_dwc3_depcmd_clear_stall(const struct device *const dev,
					struct udc_dwc3_ep_data *const ep_data,
					uint32_t flags)
{
	LOG_DBG("DepClearStall EP%02x", ep_data->cfg.addr);

	flags |= UDC_DWC3_DEPCMD_DEPCSTALL;

	/*
	 * udc_dwc3_depcmd() reports 0 for success and UDC_DWC3_XFERRSCIDX_INVALID
	 * for failure - it does NOT return the DEPCMD register, so this must not
	 * be decoded as one.
	 */
	return udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), flags) == 0U;
}

/* Defined below; needed here to release a transfer resource a rejected Start
 * Transfer could not obtain.
 */
static bool udc_dwc3_depcmd_end_xfer(const struct device *const dev,
				     struct udc_dwc3_ep_data *const ep_data,
				     uint32_t flags);

/* Defined below; End Transfer needs it to tell a lost index from no transfer. */
static bool udc_dwc3_ep_ring_outstanding(const struct udc_dwc3_ep_data *const ep_data);

/*
 * Record a transfer resource index the controller handed back.
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
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint32_t idx;
	uint32_t cmd;
	uint32_t reg;

	/*
	 * Last line of defence for databook 3.2.2.7. Nothing should arrive here
	 * with an End Transfer still concluding on this endpoint:
	 */
	if (udc_dwc3_ep_is_ending(ep_data)) {
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
	 * The value written is 8, which the databook calls Resume - "the
	 * software must write Resume (8) into the DCTL.ULStChngReq field" - not
	 * a benign no-op when the link is already up.
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

	/* INVARIANT 1: a Start Transfer only from IDLE. */
	/*
	 * CONTROL ENDPOINTS ARE EXEMPT, and the exemption is the hardware's, not a
	 * concession.
	 */
	if (USB_EP_GET_IDX(ep_data->cfg.addr) > 0U &&
	    ep_data->xfer_state != UDC_DWC3_EP_IDLE) {
		LOG_ERR("EP%02x Start Transfer refused: endpoint is %s, not idle",
			ep_data->cfg.addr,
			udc_dwc3_ep_state_name(ep_data->xfer_state));
		return false;
	}

	sys_write32(HI32((uintptr_t)ep_data->trb_buf), base + UDC_DWC3_DEPCMDPAR0(ep_data->epn));
	sys_write32(LO32((uintptr_t)ep_data->trb_buf), base + UDC_DWC3_DEPCMDPAR1(ep_data->epn));

	/*
	 * CMDIOC asks the controller for a Command Complete event carrying the
	 * transfer resource index, which is what replaces waiting for it here.
	 * The databook lists the index as returned "in the DEPCMDn register and
	 * in the Command Complete event";
	 */
	cmd = UDC_DWC3_DEPCMD_DEPSTRTXFER;

	/*
	 * CMDIOC, as the comment above always said it should be. Without it a
	 * Start Transfer has exactly one way to be observed - the synchronous
	 * poll in udc_dwc3_depcmd() - and if that misses, the outcome is
	 * unknowable and the endpoint sits in STARTING until something else
	 * issues a command on it.
	 */
	cmd |= UDC_DWC3_DEPCMD_CMDIOC;

	/*
	 * Marked BEFORE the command is posted, for the reason End Transfer arms
	 * its state first:
	 */
	(void)udc_dwc3_ep_state_set(ep_data, UDC_DWC3_EP_STARTING);

	idx = udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), cmd);

	/*
	 * Keep the previous index when the command failed. The controller only
	 * assigns a transfer resource on success, so storing what the register held
	 * after a failure would replace a working index with a value that addresses
	 * some other endpoint's resource - or none - for every Update Transfer and
	 * End Transfer that follows.
	 */
	if (idx == UDC_DWC3_XFERRSCIDX_INVALID) {
		struct udc_dwc3_data *const priv = udc_get_private(dev);

		/*
		 * STILL EXECUTING IS NOT NOT-ISSUED, AND THIS IS THE ONE THAT
		 * MATTERS. udc_dwc3_depcmd() reports XFERRSCIDX_INVALID both for
		 * a command the controller REJECTED and for one still running
		 * when the 32 us fast poll expired. Databook DEPCMD.CmdAct:
		 */
		if ((sys_read32(base + UDC_DWC3_DEPCMD(ep_data->epn)) &
		     UDC_DWC3_DEPCMD_CMDACT) != 0U) {
			LOG_WRN("EP%02x Start Transfer still executing past the poll "
				"budget: left STARTING for its Command Complete rather "
				"than resetting the endpoint under a live command",
				ep_data->cfg.addr);
			return true;
		}

		priv->ctrl_start_fail++;

		/* The command was never issued, so no resource was assigned. */
		LOG_ERR("Start Transfer not issued on EP%02x, transfer resource index "
			"is 0x%x%s (%u so far)", ep_data->cfg.addr,
			ep_data->xferrscidx,
			ep_data->xferrscidx == UDC_DWC3_XFERRSCIDX_INVALID
				? " (none established)" : " (still running)",
			priv->ctrl_start_fail);

		/*
		 * The arm DID NOT TAKE EFFECT, and returning quietly here is
		 * what turns that into a dead device. CmdStatus 4'h1 on a Start
		 * Transfer means "there is no transfer resource available on the
		 * endpoint", and 3.2.2.2 says how to get one back:
		 */
		udc_ep_set_busy(&ep_data->cfg, false);

		/*
		 * RETRY HERE, because the Command Complete that used to carry
		 * this failure no longer arrives:
		 */
		/*
		 * NO RETRY. A refused Start Transfer must not simply be re-issued.
		 */
		udc_dwc3_ep_state_reset(ep_data);

		/* Budget spent. Returning false is what the caller needs: */
		LOG_ERR("EP%02x Start Transfer refused; endpoint returned to idle for "
			"the recovery machine to act on", ep_data->cfg.addr);

		return false;
	}

	/*
	 * udc_dwc3_depcmd() has already invalidated the index. It arrives with the
	 * Command Complete event, or is read back by the next command's pre-poll
	 * if something needs it before that event has been drained.
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
	 * Only STARTING is waiting for an index. Reading one in any other state
	 * means DEPCMD still carries the result of a transfer that has since
	 * been reset - by DEPSTARTCFG, a disable or a bus reset - and adopting
	 * it would leave xferrscidx valid against an endpoint whose state says
	 * no transfer exists.
	 */
	if (ep_data->xfer_state != UDC_DWC3_EP_STARTING) {
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
	 * The index is what STARTING was waiting for. Whichever observer gets
	 * here first - the post-poll, the pre-poll of the next command on this
	 * endpoint, or the Command Complete handler - makes the same transition,
	 * and it is idempotent:
	 */
	(void)udc_dwc3_ep_state_set(ep_data, UDC_DWC3_EP_RUNNING);
}

/*
 * Take the transfer resource index from DEPCMD if it is already there.
 */
/*
 * Adopt a transfer resource index delivered in a Command Complete EVENT.
 */
static void udc_dwc3_adopt_xferrscidx_evt(const struct device *const dev,
					  struct udc_dwc3_ep_data *const ep_data,
					  const uint32_t idx)
{
	if (ep_data->xfer_state == UDC_DWC3_EP_STARTING) {
		udc_dwc3_store_xferrscidx(dev, ep_data, idx);
		(void)udc_dwc3_ep_state_set(ep_data, UDC_DWC3_EP_RUNNING);
		return;
	}

	/*
	 * ALREADY RUNNING ON THE SAME INDEX IS THE ORDINARY CASE, NOT A FAULT.
	 * udc_dwc3_depcmd()'s post-poll usually sees CmdAct clear before this
	 * event is drained, and udc_dwc3_adopt_xferrscidx() has then already
	 * taken the index out of DEPCMD and moved the endpoint STARTING ->
	 * RUNNING.
	 */
	if (ep_data->xfer_state == UDC_DWC3_EP_RUNNING) {
		if (ep_data->xferrscidx == idx) {
			return;
		}

		/*
		 * RUNNING WITH NO INDEX AT ALL. ADOPT IT - THIS EVENT IS THE ONLY
		 * PLACE IT EXISTS.
		 */
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
	 * Anything else IS the P0 case: a completion adopted against a state
	 * that says no such transfer exists - IDLE after the synchronous path
	 * gave up, ENDING under a teardown, or RUNNING on a DIFFERENT index.
	 */
	LOG_WRN_RATELIMIT("EP%02x Start Transfer completion for index %u arrived "
			  "while the endpoint is %s holding index %u: NOT adopted",
			  ep_data->cfg.addr, idx,
			  udc_dwc3_ep_state_name(ep_data->xfer_state),
			  ep_data->xferrscidx);
}

/*
 * Recover a transfer resource index from DEPCMD if one is already there.
 */
static void udc_dwc3_peek_xferrscidx(const struct device *const dev,
				     struct udc_dwc3_ep_data *const ep_data)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	const struct udc_dwc3_data *const priv = udc_get_private(dev);

	if (ep_data->xferrscidx != UDC_DWC3_XFERRSCIDX_INVALID) {
		return;
	}

	/* Undefined until this endpoint has been commanded at least once. */
	if (ep_data->epn >= UDC_DWC3_MAX_EPN || (priv->depcmd_issued & BIT(ep_data->epn)) == 0) {
		return;
	}

	udc_dwc3_adopt_xferrscidx(dev, ep_data,
				  sys_read32(base + UDC_DWC3_DEPCMD(ep_data->epn)));
}

/*
 * Returns true when the command was issued. The watchdog needs to know: it
 * reports a re-cache and counts it, and after the refusal below that claim can
 * be untrue.
 */
static bool udc_dwc3_depcmd_update_xfer(const struct device *const dev,
					struct udc_dwc3_ep_data *const ep_data)
{
	uint32_t flags = 0;

	udc_dwc3_peek_xferrscidx(dev, ep_data);

	/*
	 * REFUSE, do not proceed. This used to warn and issue the command anyway,
	 * on the reasoning that a non-control endpoint gets exactly one Start
	 * Transfer and refusing here would take the whole data path down rather
	 * than one transfer.
	 */
	if (ep_data->xferrscidx == UDC_DWC3_XFERRSCIDX_INVALID) {
		LOG_ERR("Update Transfer on EP%02x refused: no transfer resource "
			"index established", ep_data->cfg.addr);
		return false;
	}

	/*
	 * Same ordering requirement as Start Transfer, and this is the path that
	 * runs per buffer on every bulk and interrupt endpoint - the hot one.
	 */
	flags |= UDC_DWC3_DEPCMD_DEPUPDXFER;
	flags |= FIELD_PREP(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, ep_data->xferrscidx);

	/*
	 * CHECKED. This return was discarded, and it is the hot path - one
	 * Update Transfer per buffer on every bulk and interrupt endpoint.
	 */
	/*
	 * CHECKED, NOT RE-REPORTED. udc_dwc3_depcmd() has already logged the
	 * endpoint, the command and the reason - a reject status or a status it
	 * could not read - and that is the one place the failure belongs.
	 */
	if (udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), flags) ==
	    UDC_DWC3_XFERRSCIDX_INVALID) {
		return false;
	}

	/* DBG: this fires once per buffer from udc_dwc3_trb_bulk(). */
	LOG_DBG("DepUpdateXfer done EP%02x, addr 0x%08x, data 0x%08x, xferrscidx 0x%x",
		ep_data->cfg.addr, UDC_DWC3_DEPCMD(ep_data->epn), flags, ep_data->xferrscidx);

	return true;
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
	uint32_t saved_idx;

	udc_dwc3_peek_xferrscidx(dev, ep_data);

	/* Same refusal as Update Transfer, and for the same reason: */
	/*
	 * LAST RESORT: TAKE THE INDEX STRAIGHT OUT OF DEPCMD. The STARTING guard
	 * is right for a SPECULATIVE adoption - taking a stale index against an
	 * endpoint with no transfer is what leaves xferrscidx valid on an IDLE
	 * endpoint and invites O4 to restart on top of it.
	 */
	if (ep_data->xferrscidx == UDC_DWC3_XFERRSCIDX_INVALID &&
	    udc_dwc3_ep_ring_outstanding(ep_data)) {
		const uint32_t reg = sys_read32(base + UDC_DWC3_DEPCMD(ep_data->epn));

		if ((reg & UDC_DWC3_DEPCMD_CMDACT) == 0U &&
		    (reg & UDC_DWC3_DEPCMD_CMDTYP_MASK) ==
			    UDC_DWC3_DEPCMD_DEPSTRTXFER &&
		    (reg & UDC_DWC3_DEPCMD_STATUS_MASK) ==
			    UDC_DWC3_DEPCMD_STATUS_OK) {
			const uint32_t idx =
				FIELD_GET(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, reg);

			LOG_WRN("EP%02x has no stored transfer resource index but the "
				"controller still owns a descriptor: ending the "
				"transfer with index %u read back from DEPCMD",
				ep_data->cfg.addr, idx);

			flags |= FIELD_PREP(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, idx);
			flags |= UDC_DWC3_DEPCMD_DEPENDXFER;

			if (udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn),
					    flags) == 0U) {
				return true;
			}

			/* Pending, not rejected - same rule as the path below. */
			return (sys_read32(base + UDC_DWC3_DEPCMD(ep_data->epn)) &
				UDC_DWC3_DEPCMD_CMDACT) != 0U;
		}
	}
	if (ep_data->xferrscidx == UDC_DWC3_XFERRSCIDX_INVALID) {
		/*
		 * Do NOT leave ENDING here. udc_dwc3_depcmd() invalidates the
		 * index the moment it posts an End Transfer and the flag is set
		 * only afterwards, so "INVALID and pending" is exactly the
		 * window in which a FIRST End Transfer is still concluding. its
		 * Command Complete then logs as unexpected, and ep_resume()
		 * passes its 3.2.2.7 deferral check and starts a transfer while
		 * the End Transfer is still in flight.
		 */
		LOG_ERR("End Transfer on EP%02x refused: no transfer resource index "
			"established", ep_data->cfg.addr);
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
	 * KEEP THE INDEX ACROSS THE COMMAND. udc_dwc3_depcmd() invalidates
	 * xferrscidx before posting End Transfer, because a command that
	 * succeeds really does give the resource back. A REJECTED one does not:
	 * the controller refused to act, so the transfer is still running and
	 * still owns the resource - and the endpoint goes back to RUNNING below.
	 * Without restoring it, RUNNING is left paired with an INVALID index,
	 * after which Update Transfer is refused for want of a resource, a
	 * second End Transfer cannot be issued to reclaim it, and the endpoint
	 * is dead with no path back.
	 */
	saved_idx = ep_data->xferrscidx;

	/*
	 * A failed return means the command was never issued - the previous one
	 * on this endpoint was still active when the pre-poll gave up.
	 */
	if (udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), flags) ==
	    UDC_DWC3_XFERRSCIDX_INVALID) {
		/*
		 * Never issued, so nothing was ended: the transfer is still
		 * running and still holds its resource.
		 */
		/*
		 * "STILL ACTIVE" IS NOT "NOT ISSUED". udc_dwc3_depcmd() returns
		 * XFERRSCIDX_INVALID for two opposite outcomes: Databook
		 * DEPCMD.CmdAct:
		 */
		if ((sys_read32(base + UDC_DWC3_DEPCMD(ep_data->epn)) &
		     UDC_DWC3_DEPCMD_CMDACT) != 0U) {
			LOG_WRN("EP%02x End Transfer still executing past the poll "
				"budget: left ENDING for its Command Complete rather "
				"than disowning a command already in flight",
				ep_data->cfg.addr);
			return true;
		}

		/* Refused, so the resource never came back - see above. */
		if (saved_idx != UDC_DWC3_XFERRSCIDX_INVALID) {
			udc_dwc3_store_xferrscidx(dev, ep_data, saved_idx);
		}
		(void)udc_dwc3_ep_state_set(ep_data, UDC_DWC3_EP_RUNNING);
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
static void udc_dwc3_depcmd_start_config(const struct device *const dev,
					 bool is_control)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	uint32_t flags = 0;

	flags |= FIELD_PREP(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, is_control ? 0 : 2);
	flags |= UDC_DWC3_DEPCMD_DEPSTARTCFG;

	udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(0), flags);

	/*
	 * DEPSTARTCFG reassigns the controller's transfer resources, so every
	 * index handed out by an earlier Start Transfer stops being meaningful,
	 * and so does every transfer that was running, starting or ending on the
	 * strength of one.
	 */
	for (uint8_t i = 0; i < cfg->num_in_eps; i++) {
		udc_dwc3_ep_state_reset(&cfg->ep_data_in[i]);
	}
	for (uint8_t i = 0; i < cfg->num_out_eps; i++) {
		udc_dwc3_ep_state_reset(&cfg->ep_data_out[i]);
	}

	/*
	 * A NEW POOL GENERATION. Every endpoint must run DEPXFERCFG once more for
	 * it, and exactly once - see udc_dwc3_ep_resume().
	 */
	((struct udc_dwc3_data *)udc_get_private(dev))->xfercfg_epoch++;

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
	 * Silent when the caller already supplied a whole number of packets,
	 * which is the normal case:
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

/*
 * Arm one buffer in the endpoint's TRB ring and advance head.
 */
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
	 * THE UDC MUTEX IS WHAT PROTECTS THIS RING, not the work queue. head,
	 * tail, full and net_buf[] are shared with udc_dwc3_pop_trb(), and the
	 * dispatch thread is separate from udc_get_work_q(), so the queue
	 * guarantees nothing here.
	 */

	/* If the next TRB in the chain is still owned by the hardware, need
	 * to retry later when more resources become available.
	 */
	__ASSERT_NO_MSG(!ep_data->full);

	/* Associate an active buffer and a TRB together */
	ep_data->net_buf[ep_data->head] = buf;

	/*
	 * An OUT descriptor must be a whole number of packets (S4.2.3.3), so the
	 * programmed size is rounded up to MaxPacketSize.
	 */
	ep_data->n_arm++;
	ep_data->armed_len[ep_data->head] = out_size;

	udc_dwc3_trb_fill(trb, (uintptr_t)buf->data, out_size, ctrl);

	LOG_DBG("PUSH %u, buf %p, data %p, size %u -> %u",
		ep_data->head, (void *)buf, (void *)buf->data, buf->size, out_size);

	/* Per-arm trace for the non-control endpoints. Debug-only: it fires
	 * once per buffer and the console is synchronous.
	 */
	if (ep_data->cfg.addr != UDC_DWC3_TRBLOG_SKIP_EP) {
		LOG_DBG("EP%02x: ARM s%u len=%u n=%u", ep_data->cfg.addr,
			ep_data->head, out_size, ep_data->n_arm);
	}

	ep_data->head = (ep_data->head + 1) % (CONFIG_UDC_DWC3_TRB_NUM - 1);

	ep_data->full = (ep_data->head == ep_data->tail);
}

/*
 * Does the controller, or software, still own anything in this ring?
 */
static bool udc_dwc3_ep_ring_outstanding(const struct udc_dwc3_ep_data *const ep_data)
{
	if (ep_data->trb_buf == NULL) {
		return false;
	}

	for (uint32_t i = 0U; i < (CONFIG_UDC_DWC3_TRB_NUM - 1U); i++) {
		if ((ep_data->trb_buf[i].ctrl & UDC_DWC3_TRB_CTRL_HWO) != 0U) {
			return true;
		}
		if (ep_data->net_buf[i] != NULL) {
			return true;
		}
	}

	return false;
}

/*
 * Retire the oldest completed TRB. Returns -EBUSY while the controller still owns
 * it and -ENOBUFS when the slot holds no buffer.
 */
static int udc_dwc3_pop_trb(struct udc_dwc3_ep_data *const ep_data,
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

	/* Retire side of the arm trace above - the two together give arm -> retire
	 * for a slot, which is the interval the 8 s stall lives in. */
	if (ep_data->cfg.addr != UDC_DWC3_TRBLOG_SKIP_EP) {
		LOG_DBG("EP%02x: RET s%u sts=%x n=%u", ep_data->cfg.addr,
			ep_data->tail, trb->status, ep_data->n_retire);
	}

	/* -1 for link trb */
	ep_data->tail = (ep_data->tail + 1) % (CONFIG_UDC_DWC3_TRB_NUM - 1);

	/* If we just pulled a TRB, we know we made one hole and we are not full anymore */
	ep_data->full = false;

	/*
	 * Received length, counted down from the size that was PROGRAMMED, not from
	 * buf->size.
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
 * Arm an EP0-OUT stage - SETUP, data or status.
 */
static bool udc_dwc3_trb_ctrl_out(const struct device *const dev, struct net_buf *const buf,
				  const uint32_t ctrl)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_ep_data *const ep_data = &cfg->ep_data_out[0];
	volatile struct udc_dwc3_trb *const trb = ep_data->trb_buf;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	uint32_t size;

	/*
	 * Same ownership guard as udc_dwc3_trb_ctrl_in(). Overwriting a TRB with
	 * HWO set is a databook violation, and the Start Transfer that follows the
	 * overwrite is refused with "no transfer resource available".
	 */
	if ((trb[0].ctrl & UDC_DWC3_TRB_CTRL_HWO) != 0U) {
		priv->ctrl_arm_refused++;
		LOG_WRN_RATELIMIT("control OUT still owned by the controller "
				  "(ctrl 0x%08x sts 0x%08x), not arming 0x%x over it; "
				  "dropping the claim for the host's next SETUP "
				  "(refused %u, stomp %u)",
				  trb[0].ctrl, trb[0].status, ctrl,
				  priv->ctrl_arm_refused, priv->trb_stomp);
		udc_ep_set_busy(&ep_data->cfg, false);
		return false;
	}

	priv->last_xfer_type = ctrl;
	priv->last_xfer_dir = USB_EP_DIR_OUT;

	if ((ctrl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_2 ||
	     ctrl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3) &&
	    priv->ctrl_state == UDC_DWC3_CTRL_STATUS_READY) {
		/*
		 * STATUS_ARMED is only ever entered from STATUS_READY. Arming a
		 * status TRB from any other state is an out-of-flow arm (the shell
		 * diagnostics below are the one way it can happen), and recording
		 * it would insert an IDLE -> STATUS_ARMED transition the machine
		 * does not define into the wedge trace.
		 */
		udc_dwc3_ctrl_state_set(dev, UDC_DWC3_CTRL_STATUS_ARMED);
	}

	/*
	 * The TRB carries the size of the TRANSFER, not the size of the buffer
	 * that happens to hold it - and for a SETUP the spec fixes that number.
	 * SPEC, Programming Guide 3.30b, 3.1.2.2 "Setup and Status TRB
	 * Structure":
	 */
	if (ctrl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_SETUP) {
		size = sizeof(struct usb_setup_packet);
	} else if (ctrl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_2 ||
		   ctrl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3) {
		/*
		 * MaxPacketSize, not 0. This is the status stage of a control
		 * READ: The databook contradicts itself here.
		 */
		size = USB_MPS_EP_SIZE(ep_data->cfg.mps);
	} else {
		size = buf->size;
	}

	/*
	 * An OUT descriptor must be a whole number of packets: "the total size
	 * of a Buffer Descriptor must be a multiple of MaxPacketSize".
	 */
	/*
	 * cfg.mps is the ENCODED Max Packet Size: bits 10:0 are the packet size
	 * and bits 12:11 carry the additional-transactions count for high-
	 * bandwidth periodic endpoints. The databook rule is a multiple of
	 * MaxPacketSize, so the modulo has to be against the packet-size field
	 * alone - using the raw value would compute against size|(mult<<11) and
	 * produce nonsense on any endpoint that carries mult.
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

	/*
	 * REPORT WHAT ACTUALLY HAPPENED. Discarding this and returning true made
	 * the control machine record a stage as armed when the Start Transfer
	 * was refused and the endpoint reset to IDLE.
	 */
	return udc_dwc3_depcmd_start_xfer(dev, ep_data);
}


/*
 * Arm an EP0-IN stage - data or status.
 */
static bool udc_dwc3_trb_ctrl_in(const struct device *const dev,
				 struct net_buf *const buf,
				 const uint32_t ctrl)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	struct udc_dwc3_ep_data *const ep_data = &cfg->ep_data_in[0];
	volatile struct udc_dwc3_trb *const trb = ep_data->trb_buf;

	/*
	 * Do not arm over a descriptor the controller still owns. Overwriting a
	 * TRB with HWO set is a databook violation in its own right, and this
	 * driver's own note calls it a prime suspect for a controller left
	 * holding a descriptor it will not release.
	 */
	if ((trb[0].ctrl & UDC_DWC3_TRB_CTRL_HWO) != 0U) {
		/*
		 * Count it and drop the claim. Taking the descriptor back is NOT
		 * done here any more:
		 */
		priv->ctrl_arm_refused++;
		LOG_WRN_RATELIMIT("control IN still owned by the controller "
				  "(ctrl 0x%08x sts 0x%08x), not arming 0x%x over it; "
				  "dropping the claim for the host's next SETUP "
				  "(refused %u, stomp %u)",
				  trb[0].ctrl, trb[0].status, ctrl,
				  priv->ctrl_arm_refused, priv->trb_stomp);
		udc_ep_set_busy(&ep_data->cfg, false);
		return false;
	}

	priv->last_xfer_type = ctrl;
	priv->last_xfer_dir = USB_EP_DIR_IN;

	if ((ctrl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_2 ||
	     ctrl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3) &&
	    priv->ctrl_state == UDC_DWC3_CTRL_STATUS_READY) {
		/*
		 * Same guard as the OUT direction: STATUS_ARMED is only entered
		 * from STATUS_READY, so an out-of-flow status arm must not insert
		 * a transition the machine does not define into the wedge trace.
		 */
		udc_dwc3_ctrl_state_set(dev, UDC_DWC3_CTRL_STATUS_ARMED);
	}

	if (udc_ep_buf_has_zlp(buf)) {
		udc_dwc3_trb_fill(&trb[0], (uintptr_t)buf->data, buf->len,
				  ctrl | UDC_DWC3_TRB_CTRL_CHN |
				  UDC_DWC3_TRB_CTRL_HWO);

		/*
		 * The terminating zero-length TRB is not the first TRB of the
		 * data stage, so its type is Normal, not Control-Data:
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

	/* Report what actually happened - see the note in the IN path. */
	return udc_dwc3_depcmd_start_xfer(dev, ep_data);
}

/*
 * Arm one bulk/interrupt buffer and tell the controller about it. Returns -EBUSY
 * if the ring is full, in which case nothing was armed and the caller keeps the
 * buffer.
 */
static int udc_dwc3_trb_bulk(const struct device *const dev,
			     struct udc_dwc3_ep_data *const ep_data,
			     struct net_buf *const buf)
{
	uint32_t ctrl = UDC_DWC3_TRB_CTRL_IOC | UDC_DWC3_TRB_CTRL_HWO;

	/*
	 * CSP is Continue-on-Short-Packet, an OUT-endpoint control. per Table
	 * 4-8 XferComplete on an IN endpoint needs LST=1, which this driver
	 * never sets, and XFERCOMPLETE and XFERINPROGRESS are routed to the same
	 * handler anyway - see udc_dwc3_on_xfer_done_nonctrl().
	 */
	if (!USB_EP_DIR_IS_IN(ep_data->cfg.addr)) {
		ctrl |= UDC_DWC3_TRB_CTRL_CSP;
	}

	/*
	 * DBG for the same reason the control stages are: one line per transfer
	 * on a data endpoint is the pattern that capped control traffic at
	 * ~41/s.
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

	/*
	 * PROPAGATED. The descriptor is already in the ring with HWO set by the
	 * time this runs, so a failed Update Transfer does not merely lose a
	 * command - it leaves a buffer the controller owns and will never fetch,
	 * and the endpoint is finished until something tears it down.
	 */
	/* ...AND BECAUSE IT IS NOT UNWOUND, THIS MUST REPORT SUCCESS. */
	if (!udc_dwc3_depcmd_update_xfer(dev, ep_data)) {
		/*
		 * THE DESCRIPTOR IS ARMED AND THE CONTROLLER HAS NOT BEEN TOLD.
		 * The buffer deliberately stays owned by net_buf[] - returning
		 * an error here after udc_dwc3_push_trb() has armed it is what
		 * handed one buffer to both net_buf[] and the stack's queue and
		 * panicked the device on a double free.
		 */
		/*
		 * COUNTED, NOT RE-LOGGED. udc_dwc3_depcmd_update_xfer() already
		 * emits a LOG_ERR naming the endpoint and the reason, and this
		 * is NOT a rare path:
		 */
		ep_data->n_update_refused++;
	}

	/*
	 * last_xfer_type is NOT set here. It names the current CONTROL stage and
	 * is read as one - udc_dwc3_recover() copies it into watchdog_type when
	 * it has no guarded endpoint.
	 */
	return 0;
}

/*
 * Control buffers
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
		if (!udc_dwc3_trb_ctrl_in(dev, buf,
					  UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA)) {
			return false;
		}
		udc_dwc3_ctrl_arm_watchdog(dev, true, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA);
	} else if (bi.status && setup->wLength == 0) {
		/*
		 * An IN status stage SENDS the zero-length packet, so its TRB
		 * carries BUFSIZ 0.
		 */
		buf->size = 0;
		buf->len = 0;
		LOG_DBG("trb IN_STATUS_2 ln=%d d=%p", buf->len, (void *)buf->data);
		if (!udc_dwc3_trb_ctrl_in(dev, buf,
					  UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_2)) {
			return false;
		}
		udc_dwc3_ctrl_arm_watchdog(dev, true, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_2);
	} else if (bi.status) {
		/* Same as the two-stage case above: buf->len is what reaches the TRB. */
		buf->size = 0;
		buf->len = 0;
		LOG_DBG("trb IN_STATUS_3 ln=%d d=%p", buf->len, (void *)buf->data);
		if (!udc_dwc3_trb_ctrl_in(dev, buf,
					  UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3)) {
			return false;
		}
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
		if (!udc_dwc3_trb_ctrl_out(dev, buf,
					   UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_SETUP)) {
			return false;
		}

		udc_dwc3_ctrl_arm_watchdog(dev, false,
					   UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_SETUP);

		/* The SETUP watchdog is armed, but it cannot act on age alone. */
	} else if (bi.data) {
		LOG_DBG("trb OUT_DATA sz=%d d=%p", buf->size, (void *)buf->data);
		if (!udc_dwc3_trb_ctrl_out(dev, buf,
					   UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA)) {
			return false;
		}
		udc_dwc3_ctrl_arm_watchdog(dev, false, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA);
	} else if (bi.status) {
		/*
		 * buf->size is deliberately NOT zeroed here any more. The status
		 * OUT buffer is allocated at bMaxPacketSize0 by
		 * udc_ctrl_status_alloc(), and udc_dwc3_trb_ctrl_out() now
		 * programs MaxPacketSize for this stage - see the reasoning
		 * there.
		 */
		LOG_DBG("trb OUT_STATUS_3 sz=%d d=%p", buf->size, (void *)buf->data);
		if (!udc_dwc3_trb_ctrl_out(dev, buf,
					   UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3)) {
			return false;
		}
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
/*
 * Arm a control stage if both EP0 halves are free, else decline and record why.
 */
static void udc_dwc3_ctrl_try(const struct device *const dev,
			      struct udc_dwc3_ep_data *ep_data);

/*
 * Did the controller retire this control endpoint's TRB because the host
 * started a new SETUP?
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
		 * First few only. If this turns out to be routine rather than
		 * rare, a line per occurrence is exactly the flood that took the
		 * event ring down before;
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
	 * Rate limited, because this fires once per abandoned transfer and a
	 * host that floods control requests while streaming abandons them
	 * continuously.
	 */
	LOG_WRN_RATELIMIT("host started a new SETUP during a control stage on EP%02x, "
			  "abandoning the transfer in progress (%u so far)",
			  ep_data->cfg.addr, priv->ctrl_setup_pending);

	/* Drain this endpoint, and the other one only if it is idle. */
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
	 * skipping a data stage because a new SETUP arrived:
	 */
	udc_dwc3_fifo_flush_tx(dev, cfg->ep_data_in[0].cfg.addr & 0x7fU);

	if (udc_ep_is_busy(&peer->cfg)) {
		LOG_DBG("EP%02x is busy with the replacement transfer, "
			"leaving it to its own completion", peer->cfg.addr);

		/*
		 * REVERTED. This used to call udc_dwc3_ctrl_try() on EP0-OUT
		 * here, on the reasoning that abandoning must not leave the
		 * control endpoint unarmed.
		 */
		return;
	}

	udc_dwc3_ctrl_drain_abandoned(dev, peer);
	udc_ep_set_busy(&peer->cfg, false);

	/* The abandoned transfer is gone, so its stage goes with it. */
	udc_dwc3_ctrl_state_set(dev, UDC_DWC3_CTRL_IDLE);

	/*
	 * Release both control endpoints. Only one of them owned a TRB, but the
	 * claim is taken as a pair for data and status stages, so both are cleared
	 * before the replacement SETUP tries to claim its own.
	 */
	/*
	 * Arm the queued SETUP directly rather than going through
	 * udc_dwc3_ctrl_next().
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

/*
 * Arm a control stage if both EP0 halves are free, else decline and record why.
 */
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
	 * The busy test and the claim below are not atomic on their own, and
	 * this function is reached from two places:
	 */
	buf = udc_buf_peek(&ep_data->cfg);
	if (buf == NULL) {
		/*
		 * DBG, not INF: in steady state "nothing queued right now" is
		 * the normal answer, and at INF it was 36% of one capture - 2.4
		 * MB, about 258 s of console time in a 638 s run.
		 */
		LOG_DBG("EP%02X: no buf", ep_data->cfg.addr);
		return;
	}

	/*
	 * Databook 3.2.2.7 again, on the control side. Arming a stage here ends in
	 * udc_dwc3_depcmd_start_xfer(), which must not run while this endpoint's
	 * End Transfer is still concluding.
	 */
	if (udc_dwc3_ep_is_ending(ep_data)) {
		priv->ctrl_deferred_arm++;

		/*
		 * Put a deadline on the deferral. Everything else on the control path
		 * is watched from the moment it arms, inside
		 * udc_dwc3_trb_ctrl_in()/_out(), so a stage that never gets that far
		 * would be the one thing here with no timeout behind it - and the
		 * event that releases it is exactly the Endpoint Command Complete this
		 * driver is chasing for going missing.
		 */
		/*
		 * Record the endpoint, or udc_dwc3_on_ctrl() cannot recognise
		 * this deadline as its own and will leave it pending for ever.
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
	 * 4.4.1 step 3 / 4.4.2 step 4: the status stage waits for its
	 * XferNotReady.
	 */
	if (udc_get_buf_info(buf)->status &&
	    priv->ctrl_state < UDC_DWC3_CTRL_STATUS_READY) {
		priv->ctrl_status_defer++;

		if (priv->watchdog_ep == NULL) {
			priv->watchdog_ep = ep_data;
			priv->watchdog_type = UDC_DWC3_WATCHDOG_TYPE_NONE;
		}

		k_work_schedule(&priv->watchdog_dwork,
				K_MSEC(CONFIG_UDC_DWC3_RECOVERY_TIMEOUT));

		LOG_DBG("EP%02X: status stage held until XferNotReady(Status)",
			ep_data->cfg.addr);
		return;
	}

	/*
	 * A SETUP is checked against ITS OWN endpoint only; every other stage has
	 * to wait for the pair.
	 */
	if (udc_get_buf_info(buf)->setup) {
		if (udc_ep_is_busy(&ep_data->cfg)) {
			LOG_DBG("EP%02X: busy (SETUP)", ep_data->cfg.addr);
			priv->ctrl_decline++;
			priv->ctrl_decline_pending = true;
			priv->ctrl_decline_t = k_cycle_get_32();
			return;
		}
	} else if (udc_ep_is_busy(&ep_data->cfg)) {
		LOG_DBG("EP%02X: busy", ep_data->cfg.addr);
		priv->ctrl_decline++;
		priv->ctrl_decline_pending = true;
		priv->ctrl_decline_t = k_cycle_get_32();
		return;
	} else if (udc_ep_is_busy(&peer->cfg) && !udc_dwc3_ctrl_armed_setup(peer)) {
		LOG_DBG("ctrl eps busy");
		priv->ctrl_decline++;
		priv->ctrl_decline_pending = true;
		priv->ctrl_decline_t = k_cycle_get_32();
		return;
	}

	/* A claim was granted, so the control path is moving. */
	priv->ctrl_arm_t0 = k_cycle_get_32();
	priv->ctrl_decline_pending = false;

	udc_ep_set_busy(&ep_data->cfg, true);

	/*
	 * Give the claim back if nothing was armed. Two kinds of refusal reach
	 * here: an unusable buffer type, and a TRB the controller still owns -
	 * arming over HWO is a databook violation and the Start Transfer after
	 * it is refused for want of a resource.
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
 * Arm whatever control stage the current state calls for next.
 */
static void udc_dwc3_ctrl_next(const struct device *const dev)
{
	//struct udc_dwc3_data *const priv = udc_get_private(dev);
	const struct udc_dwc3_config *const cfg = dev->config;

	LOG_DBG("load");

	/* Offer BOTH control endpoints rather than computing which is next. */
	udc_dwc3_ctrl_try(dev, &cfg->ep_data_in[0]);
	udc_dwc3_ctrl_try(dev, &cfg->ep_data_out[0]);
}

#include "../../subsys/usb/device_next/usbd_ch9.h"

/*
 * UDC API: cancel queued buffers on an endpoint.
 */
static int udc_dwc3_ep_dequeue(const struct device *const dev,
			       struct udc_ep_config *const ep_cfg);
static int udc_dwc3_disable(const struct device *const dev);
static int udc_dwc3_enable(const struct device *const dev);
static int udc_dwc3_init(const struct device *const dev);
static int udc_dwc3_ep_enable(const struct device *const dev, struct udc_ep_config *const ep_cfg);

/*
 * Second half of a control-endpoint recovery.
 */

/* The CORE debug registers worth comparing healthy against wedged. */
struct udc_dwc3_core_dbg {
	uint32_t ltssm;
	uint32_t bmu;
	uint32_t lnmcc;
	uint32_t lsp;
	uint32_t epinfo0;
	uint32_t epinfo1;
};

/*
 * Dump the controller's own view of itself.
 */
static void udc_dwc3_core_dbg_read(const mm_reg_t base,
				   struct udc_dwc3_core_dbg *const s)
{
	/*
	 * GDBGLSP is a muxed window, so select the source before reading it. do
	 * not decode GDBGLSP until the device-mode selector encoding is
	 * confirmed against the databook.
	 */
	sys_write32(0U, base + UDC_DWC3_GDBGLSPMUX_DEV);

	s->ltssm   = sys_read32(base + UDC_DWC3_GDBGLTSSM);
	s->bmu     = sys_read32(base + UDC_DWC3_GDBGBMU);
	s->lnmcc   = sys_read32(base + UDC_DWC3_GDBGLNMCC);
	s->lsp     = sys_read32(base + UDC_DWC3_GDBGLSP);
	s->epinfo0 = sys_read32(base + UDC_DWC3_GDBGEPINFO0);
	s->epinfo1 = sys_read32(base + UDC_DWC3_GDBGEPINFO1);
}

/*
 * Sample the core debug registers into the log.
 */
static void udc_dwc3_core_dbg_log(const char *const tag,
				  const struct udc_dwc3_core_dbg *const s)
{
	LOG_INF("  CORE%s: GDBGLTSSM=0x%08x GDBGBMU=0x%08x GDBGLNMCC=0x%08x "
		"GDBGLSP=0x%08x GDBGEPINFO=0x%08x_%08x",
		tag, s->ltssm, s->bmu, s->lnmcc, s->lsp, s->epinfo1, s->epinfo0);
}

/*
 * Dump the controller's own view of itself - passive register reads.
 */
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

		/* Queue 0: the per-endpoint RxFIFO view is already dumped above. */
		r |= FIELD_PREP(UDC_DWC3_GDBGFIFOSPACE_QUEUENUM_MASK, 0U);
		sys_write32(r, base + UDC_DWC3_GDBGFIFOSPACE);
		r = sys_read32(base + UDC_DWC3_GDBGFIFOSPACE);

		LOG_INF("  CORE: %-9s space=%u (raw 0x%08x)", queues[i].name,
			(uint32_t)FIELD_GET(UDC_DWC3_GDBGFIFOSPACE_AVAILABLE_MASK, r),
			r);
	}
}

/*
 * Controller-level recovery of a stuck control transfer.
 */
static int udc_dwc3_recover(const struct device *dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	struct udc_dwc3_ep_data *ep_data;

	LOG_WRN("Recovering USB state");
	/* Take the UDC mutex, not the scheduler lock. */
	udc_lock_internal(dev, K_FOREVER);


	/*
	 * A control endpoint still marked as concluding an End Transfer, with no
	 * recovery of our own outstanding, means its Endpoint Command Complete
	 * never arrived.
	 */
	{
		struct udc_dwc3_ep_data *const ctrl[2] = {
			&cfg->ep_data_in[0], &cfg->ep_data_out[0],
		};
		bool released = false;

		for (int i = 0; i < 2; i++) {
			if (!udc_dwc3_ep_is_ending(ctrl[i])) {
				continue;
			}

			LOG_ERR("no Endpoint Command Complete arrived for the End Transfer "
				"on EP%02x, releasing it (deferred arms so far: %u)",
				ctrl[i]->cfg.addr, priv->ctrl_deferred_arm);

			udc_dwc3_ep_state_reset(ctrl[i]);
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
	 * End the stuck transfer and stop here. The re-arm was to have been
	 * driven by this End Transfer's own completion event; The databook names
	 * that deadlock and gives this as the way out:
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
	 * recover" typed at the shell reaches here too, and a person asking for
	 * a recovery expects one.
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
	 * udc_dwc3_wait_cmdact_zero().
	 */

	/*
	 * Set Stall, NOT End Transfer. Set Stall is what the databook prescribes
	 * for control resynchronisation (4.4.1/4.4.2 error cases:
	 */
	udc_dwc3_depcmd_set_stall(dev, &cfg->ep_data_out[0]);


	/*
	 * Set Stall on its own is enough when the stage simply needs abandoning
	 * - A capture shows four in a row rescued that way.
	 */
	/* NO End Transfer on the control pair here, and no "reclaim". */

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
 */
/*
 * THE ONE PLACE THAT PUTS THE CONTROL PAIR BACK ON STEP 1.
 */
/* Defined below; teardown needs it to move armed buffers off the ring. */
static void udc_dwc3_ep_ring_release(struct udc_dwc3_ep_data *const ep_data);

/*
 * Hand every buffer parked on requeue_fifo back to the stack.
 *
 * udc_dwc3_ep_ring_release() MOVES armed buffers onto that fifo; on a teardown
 * nothing will ever resume the endpoint to re-arm them, and udc_dwc3_ep_resume()
 * is the only other consumer, so without this they are never unref'd, never
 * reported, invisible to udc_ep_cancel_queued(), and gone from a fixed pool.
 */
static void udc_dwc3_ep_return_parked(const struct device *const dev,
				      struct udc_dwc3_ep_data *const ep_data,
				      const int status)
{
	for (;;) {
		struct net_buf *parked =
			k_fifo_get(&ep_data->requeue_fifo, K_NO_WAIT);

		if (parked == NULL) {
			return;
		}

		udc_submit_ep_event(dev, parked, status);
	}
}

static void udc_dwc3_ctrl_reset_to_step1(const struct device *const dev,
					 const bool all_endpoints)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const uint32_t now = k_cycle_get_32();

	udc_dwc3_ctrl_state_set(dev, UDC_DWC3_CTRL_IDLE);

	/* The claims. Releasing these is not cosmetic: */
	udc_ep_set_busy(&cfg->ep_data_out[0].cfg, false);
	udc_ep_set_busy(&cfg->ep_data_in[0].cfg, false);

	/*
	 * And the transfer state with them. Releasing the claim while leaving
	 * the endpoint in STARTING or ENDING would make this a reset that does
	 * not reset:
	 */
	udc_dwc3_ep_state_reset(&cfg->ep_data_out[0]);
	udc_dwc3_ep_state_reset(&cfg->ep_data_in[0]);

	/*
	 * The heartbeat's stuck-claim detector. ctrl_decline_pending is a level,
	 * cleared only on a grant, so it outlives the transfer it describes
	 * unless it is cleared here.
	 */
	priv->ctrl_decline_pending = false;
	priv->ctrl_decline_t = now;
	priv->ctrl_arm_t0 = now;
	priv->ctrl_quiet_t0 = now;

	/* The per-stage watchdog, and the slot it would have acted on. */
	k_work_cancel_delayable(&priv->watchdog_dwork);
	priv->watchdog_ep = NULL;
	priv->watchdog_type = UDC_DWC3_WATCHDOG_TYPE_NONE;

	/*
	 * The watchdog's epoch marks. These are compared against the cumulative
	 * ctrl_setup_done / ctrl_status_done to allow one attempt per episode.
	 */
	priv->ctrl_wd_upd_mark = priv->ctrl_setup_done + priv->ctrl_status_done;
	priv->ctrl_setup_wd_upd_mark = priv->ctrl_wd_upd_mark;
	priv->ctrl_setup_wd_snap_setup = priv->ctrl_setup_done;
	priv->ctrl_setup_wd_snap_nonctrl = priv->nonctrl_done;

	/*
	 * Sentinel for the DATA/STATUS recovery escalation: nothing has issued a
	 * Set Stall for the episode that is about to begin, so the watchdog must
	 * not escalate on its first recovery even if the stage total happens to
	 * read zero (as it does after a power-on reset).
	 */
	priv->ctrl_recover_mark = UINT32_MAX;

	if (!all_endpoints) {
		return;
	}

	for (int i = 0; i < cfg->num_in_eps; i++) {
		udc_dwc3_ep_state_reset(&cfg->ep_data_in[i]);

		if (i > 0) {
			/*
			 * RELEASE THEN RETURN, IN THAT ORDER. ep_ring_release() is
			 * what moves armed buffers onto requeue_fifo; draining the
			 * fifo first drains an empty one and strands them. It also
			 * clears net_buf[]/head/tail/full, without which
			 * ep_ring_outstanding() keeps reporting the ring live on an
			 * endpoint this function has just set IDLE - the pair
			 * observation O4 answers by taking a second transfer
			 * resource.
			 */
			udc_dwc3_ep_ring_release(&cfg->ep_data_in[i]);
			udc_dwc3_ep_return_parked(dev, &cfg->ep_data_in[i],
						  -ECONNRESET);
		}
	}
	for (int i = 0; i < cfg->num_out_eps; i++) {
		udc_dwc3_ep_state_reset(&cfg->ep_data_out[i]);

		if (i > 0) {
			/* Release then return - see the IN loop above. */
			udc_dwc3_ep_ring_release(&cfg->ep_data_out[i]);
			udc_dwc3_ep_return_parked(dev, &cfg->ep_data_out[i],
						  -ECONNRESET);
		}
	}
}

/*
 * Reset every endpoint's transfer state - bus reset, disconnect and teardown.
 */
static void udc_dwc3_drop_xfer_state(const struct device *const dev,
				     const char *const reason)
{
	LOG_DBG("dropping outstanding End Transfer state (%s)", reason);

	udc_dwc3_ctrl_reset_to_step1(dev, true);
}

/*
 * Core soft reset (DCTL.CSFTRST) and full event-ring reinitialisation.
 */
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
	 * Bounded. This used to be a bare spin with no timeout and no yield, so
	 * a controller that never cleared CSftRst hung the driver here for good
	 * - during init, before any of the recovery machinery exists, with
	 * nothing able to report it.
	 */
	/*
	 * PHASE 1: bounded reads, no delay - the core normally clears CSftRst
	 * well inside this. PHASE 2: SLEEPS, because this is reachable at
	 * runtime, not only at boot - udc_dwc3_setup_stuck_reset() -> init() ->
	 * here, on the work queue with the UDC mutex held. A non-yielding spin
	 * there starved every thread including the drain for the whole wait.
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

	/*
	 * The core has just been reset, so no command issued before it can still
	 * report completion.
	 */
	udc_dwc3_drop_xfer_state(dev, "soft reset");

	/*
	 * The endpoint command registers go back to being undefined on read, and no
	 * command issued before the reset can still be active, so the next command
	 * on each endpoint must skip the pre-poll again. See priv->depcmd_issued.
	 */
	DEV_DATA(dev)->depcmd_issued = 0;

	/*
	 * Same reasoning for the transfer resource indices. A Start Transfer issued
	 * before the reset can no longer report, and DEPCMD reads undefined until
	 * this driver writes it again, so nothing may be collected from either
	 * source afterwards.
	 */
	for (uint8_t i = 0; i < DEV_CFG(dev)->num_in_eps; i++) {
		udc_dwc3_ep_state_reset(&DEV_CFG(dev)->ep_data_in[i]);
	}
	for (uint8_t i = 0; i < DEV_CFG(dev)->num_out_eps; i++) {
		udc_dwc3_ep_state_reset(&DEV_CFG(dev)->ep_data_out[i]);
	}

	/*
	 * SoC bus configuration. Register hygiene, not a fix: the combination
	 * the bitfile powers up with is one the databook does not define, and
	 * two independent vendor trees here ship the same GSBUSCFG0 for this
	 * controller.
	 */
	LOG_INF("BUSCFG at reset: GSBUSCFG0=0x%08x GSBUSCFG1=0x%08x",
		sys_read32(base + UDC_DWC3_GSBUSCFG0),
		sys_read32(base + UDC_DWC3_GSBUSCFG1));

	/*
	 * The three registers this driver leaves entirely at their power-on
	 * values and could not, until now, observe. the databook's CSftRst
	 * exception list (quoted where the soft reset is issued) names them
	 * among the registers a core soft reset does NOT clear.
	 */
	LOG_INF("POR unpinned: GUSB2PHYCFG=0x%08x GUSB3PIPECTL=0x%08x GTXTHRCFG=0x%08x",
		sys_read32(base + UDC_DWC3_GUSB2PHYCFG),
		sys_read32(base + UDC_DWC3_GUSB3PIPECTL),
		sys_read32(base + UDC_DWC3_GTXTHRCFG));

	sys_write32(UDC_DWC3_GSBUSCFG0_INCR16BRSTENA |
		    UDC_DWC3_GSBUSCFG0_INCR8BRSTENA |
		    UDC_DWC3_GSBUSCFG0_INCR4BRSTENA,
		    base + UDC_DWC3_GSBUSCFG0);

	/*
	 * GSBUSCFG1 NOT WRITTEN - PipeTransLimit left at power-on (3 on this part).
	 */

	LOG_INF("BUSCFG programmed: GSBUSCFG0=0x%08x GSBUSCFG1=0x%08x",
		sys_read32(base + UDC_DWC3_GSBUSCFG0),
		sys_read32(base + UDC_DWC3_GSBUSCFG1));

	/*
	 * Global Rx Threshold: disable multi-packet RX thresholding. Databook
	 * 1.2.4 erratum:
	 */
	reg = sys_read32(base + UDC_DWC3_GRXTHRCFG);
	/*
	 * The bus/DMA configuration, read once and never before logged. the
	 * reading it recorded is kept because it is still the argument for
	 * writing them at all - with no INCR burst enabled the databook says
	 * every DMA falls back to the largest enabled length, i.e.
	 */

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
	 */
	sys_clear_bits(base + UDC_DWC3_GUSB2PHYCFG,
		       UDC_DWC3_GUSB2PHYCFG_ULPIAUTORES);
	/*
	 * One-shot FIFO map. GRXFIFOSIZ0 and GTXFIFOSIZn partition a pool whose
	 * total is fixed at synthesis (GHWPARAMS7.RAM1_DEPTH);
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
	 */
	priv->evt_next = 0;
	udc_dwc3_drain_reset(&priv->drain);
	/* Both stamps are valid from here, so no "is it meaningful yet" flags. */
	priv->evt_worker_exit_t0 = k_cycle_get_32();
	priv->evt_force_t0 = k_cycle_get_32();
	/* Prime every slot before the controller is told where the buffer is. */
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

		/* Program the moderation interval. Table 1-90: */
		sys_write32(FIELD_PREP(UDC_DWC3_DEV_IMOD_DEVICE_IMODI_MASK,
				       UDC_DWC3_DEV_IMOD_INTERVAL_1MS),
			    base + UDC_DWC3_DEV_IMOD(0));
		LOG_INF("DEV_IMOD programmed to 0x%08x (IMODI=%u = %u us)",
			sys_read32(base + UDC_DWC3_DEV_IMOD(0)),
			UDC_DWC3_DEV_IMOD_INTERVAL_1MS,
			UDC_DWC3_DEV_IMOD_INTERVAL_1MS / 4U);
	}

	/*
	 * Report the address and whether it actually satisfies the size-
	 * alignment rule.
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
	 * VNDRDEVTSTRCVED IS DELIBERATELY NOT ENABLED. The event is ignored when
	 * it arrives, so enabling it bought nothing, and the databook says not
	 * to use the feature anyway:
	 */
	/*
	 * EvntOverflowEn, CmdCmpltEn and InactTimeoutRcvedEn are NOT fields in
	 * this controller's DEVTEN and used to be set here.
	 */
	reg |= UDC_DWC3_DEVTEN_ERRTICERREN;
	reg |= UDC_DWC3_DEVTEN_HIBERNATIONREQEVTEN;
	reg |= UDC_DWC3_DEVTEN_WKUPEVTEN;
	/*
	 * Link state change events are ENABLED.  With USB Reset and Connection Done
	 * they are the controller vendor's recommended minimum DEVTEN set.
	 */
	reg |= UDC_DWC3_DEVTEN_ULSTCNGEN;
	reg |= UDC_DWC3_DEVTEN_CONNECTDONEEN;
	reg |= UDC_DWC3_DEVTEN_USBRSTEN;
	reg |= UDC_DWC3_DEVTEN_DISCONNEVTEN;
	sys_write32(reg, base + UDC_DWC3_DEVTEN);

	/* Configure control endpoints */
	udc_dwc3_depcmd_start_config(dev, true);
}

/*
 * USBRST handler: return the device to the default state.
 */
static void udc_dwc3_on_usb_reset(const struct device *const dev)
{
	LOG_DBG("Going through DWC3 reset logic");

	/*
	 * A bus reset starts a new configuration: the next endpoint enable must
	 * be allowed to reassign the transfer-resource pool again. first_ep goes
	 * with it - a later session may enable a different endpoint first.
	 */
	{
		struct udc_dwc3_data *const p = udc_get_private(dev);

		p->cfg_pool_assigned = false;
		p->first_ep = 0U;
	}

	udc_dwc3_drop_xfer_state(dev, "USB reset");

	/* TODO: wait that all transfers did complete (if needed) */

	/* Perform the USB reset operations manually to improve latency */
	/* TODO: do after endpoints are configured? */
	udc_dwc3_set_address(dev, 0);
}

/*
 * CONNECTDONE handler: adopt the negotiated speed and resize EP0.
 */
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
	 * GTXFIFOSIZn is deliberately left alone, and the databook is explicit
	 * that this is the normal case:
	 */

	/* After successful speed negotiation, DWC3 sends a CONNECT_DONE event.
	 * Then only the speed-related registers are populated, and we can
	 * report the "reset" event (instead of during USB_RESET).
	 */
	udc_submit_event(dev, UDC_EVT_RESET, 0);
}

/*
 * SetConfiguration/SetInterface: end active transfers and reassign the resource
 * pool.
 */
static void udc_dwc3_on_set_config_or_interface(const struct device *const dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;

	LOG_DBG("SetConfiguration or SetInterface extra init");

	for (int i = 1; i < cfg->num_in_eps; i++) {
		/* Ring ownership, not the busy claim - see ep_ring_outstanding(). */
		if (udc_dwc3_ep_ring_outstanding(&cfg->ep_data_in[i])) {
			/*
			 * The endpoint is being ended so the transfer resource
			 * can be reassigned below.
			 */
			udc_dwc3_depcmd_end_xfer(dev, &cfg->ep_data_in[i], 0);
		}
	}
	for (int i = 1; i < cfg->num_out_eps; i++) {
		if (udc_dwc3_ep_ring_outstanding(&cfg->ep_data_out[i])) {
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
	 * Every site that abandons a control transfer zeroes trb_cache[0], and
	 * TRBCTL has no zero encoding - SPEC 6.3 Table "TRB Control (TRBCTL)"
	 * defines 1..9 (1 Normal, 2 Control-Setup, 3 Control-Status-2, 4
	 * Control-Status-3, 5 Control-Data, 6 Isochronous-First, 7 Isochronous,
	 * 8 Link, 9 Normal-ZLP) and no 0 - so a zero type can only be a cache
	 * that was cleared, never a transfer that really completed.
	 */
	if (trb_trbctl == 0U) {
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
	} else if (trb_trbctl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA) {
		LOG_HEXDUMP_DBG(buf->data, buf->len, "CTRL DATA packet sent");
		/* 4.4.2 step 5 needs to know the data stage is behind us. */
		udc_dwc3_ctrl_state_set(dev, UDC_DWC3_CTRL_DATA_DONE);
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
	 * Every site that abandons a control transfer zeroes trb_cache[0], and
	 * TRBCTL has no zero encoding - SPEC 6.3 Table "TRB Control (TRBCTL)"
	 * defines 1..9 (1 Normal, 2 Control-Setup, 3 Control-Status-2, 4
	 * Control-Status-3, 5 Control-Data, 6 Isochronous-First, 7 Isochronous,
	 * 8 Link, 9 Normal-ZLP) and no 0 - so a zero type can only be a cache
	 * that was cleared, never a transfer that really completed.
	 */
	if (trb_trbctl == 0U) {
		LOG_DBG("discarding superseded completion on EP%02x",
			ep_data->cfg.addr);
		return;
	}


	/* A new SETUP can arrive during any stage, not only the status-IN one. */
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
		 * Step 2 has happened: the SETUP retired and setup_packet
		 * describes the request now in progress.
		 */
		/* A new request starts here. One assignment: */
		udc_dwc3_ctrl_state_set(dev, UDC_DWC3_CTRL_SETUP_DONE);
		priv->ctrl_setup_done++;
		buf->len = 0;

		/*
		 * Stamp here, not in udc_dwc3_on_ctrl_in(): a SETUP retires on
		 * EP0-OUT.
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

		/* The whole SETUP as one 16-digit value instead of a hexdump: */
		const uint8_t *const sp = (const uint8_t *)&priv->setup_packet;

		/*
		 * The one line a healthy control transfer prints, and the only
		 * context an error line needs:
		 */
#ifdef UDC_DWC3_LOG_EVERY_SETUP
		{
			/* Collapse runs of the SAME SETUP packet. */
			static uint64_t last_sp;
			static uint32_t rep;
			const uint64_t this_sp =
				((uint64_t)sp[0] << 56) | ((uint64_t)sp[1] << 48) |
				((uint64_t)sp[2] << 40) | ((uint64_t)sp[3] << 32) |
				((uint64_t)sp[4] << 24) | ((uint64_t)sp[5] << 16) |
				((uint64_t)sp[6] << 8)  |  (uint64_t)sp[7];

			if (this_sp == last_sp) {
				if ((++rep % 1024U) == 0U) {
					LOG_DBG("SETUP %016llx x%u", this_sp, rep);
				}
			} else {
				if (rep != 0U) {
					LOG_DBG("SETUP %016llx x%u end", last_sp, rep);
					rep = 0U;
				}
				LOG_DBG("SETUP %016llx", this_sp);
				last_sp = this_sp;
			}
		}
#else
		(void)sp;
#endif
		udc_setup_received(dev, &priv->setup_packet);

		/*
		 * udc_setup_received() has just invalidated whatever the IN
		 * endpoint was doing - it drains that queue and releases the
		 * endpoint - but the controller has already retired that TRB and
		 * its completion is still queued behind this event.
		 */
		if (udc_dwc3_ctrl_setup_pending(dev, &cfg->ep_data_in[0])) {
			memset((void *)&cfg->ep_data_in[0].trb_buf[0], 0x00,
			       sizeof(cfg->ep_data_in[0].trb_buf[0]));
			memset((void *)&cfg->ep_data_in[0].trb_buf[1], 0x00,
			       sizeof(cfg->ep_data_in[0].trb_buf[1]));
			/*
			 * ep_data_in[0], not ep_data. This runs in
			 * udc_dwc3_on_ctrl_out(), so ep_data is the OUT
			 * endpoint;
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
			udc_dwc3_ctrl_state_set(dev, UDC_DWC3_CTRL_DATA_DONE);
		} else if (trb_trbctl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3 ||
			   trb_trbctl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_2) {
			/* STATUS_2 is accepted here as well as STATUS_3. */
			buf->len = 0;
			priv->ctrl_status_done++;
			LOG_HEXDUMP_DBG(buf->data, buf->len, "CTRL STATUS received");
		} else {
			LOG_ERR("Unexpected OUT packet type: 0x%x", trb_trbctl);
		}

		udc_submit_ep_event(dev, buf, 0);
	}

	memset(&ep_data->trb_buf[0], 0x00, sizeof(ep_data->trb_buf[0]));

	/* Defensive: slot 1 is not armed on this endpoint any more. */
	memset(&ep_data->trb_buf[1], 0x00, sizeof(ep_data->trb_buf[1]));
	memset(&ep_data->trb_cache[0], 0x00, sizeof(ep_data->trb_cache[0]));

	/* Used when receiving a completed buffer from the hardware: mark as free */
	udc_ep_set_busy(&ep_data->cfg, false);

	udc_dwc3_ctrl_next(dev);
}

/*
 * Dispatch a control completion to the handler for the endpoint it came from.
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
	 * just completed.
	 */
	if (priv->watchdog_ep == completed) {
		k_work_cancel_delayable(&priv->watchdog_dwork);

		/*
		 * Forget what the watchdog was guarding as well as cancelling
		 * it:
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
 * Required after an aborted control IN transfer. Section 4.4.2 step 8, on a
 * SETUP arriving mid-transfer: "Software has to reclaim the TRBs with HWO=1 in
 * the skipped TRBs and flush the TxFIFO." Reclaiming alone is not enough - the
 * bytes the controller had already staged for the skipped IN stage stay in the
 * FIFO and would be transmitted at the head of the next one.
 */
/*
 * Wait for DGCMD.CmdAct to clear, bounded.
 *
 * SPEC, Programming Guide 3.30b, DGCMD bit 10 CMDACT: "The software sets this
 * bit to 1 to enable the device controller to execute the generic command. The
 * device controller sets this bit to 0 after executing the command."
 */
static bool udc_dwc3_dgcmd_wait_idle(const mm_reg_t base)
{
	uint32_t polls = 0;

	/*
	 * A BOUNDED READ LOOP, WITH NO DELAY. This runs on the drain thread with
	 * the UDC mutex held, and the only non-yielding busy-waits this driver
	 * permits are the CmdAct post-poll and the event drain's 16-poll arrival
	 * phase. A k_busy_wait(1) per poll made this up to a millisecond, and
	 * udc_dwc3_fifo_flush_tx() calls it twice. A generic command retires in
	 * microseconds, so reads alone cover it; a command that has not retired
	 * within this many reads is reported, not waited on.
	 */
	while ((sys_read32(base + UDC_DWC3_DGCMD) & UDC_DWC3_DGCMD_ACT) != 0U) {
		if (++polls >= UDC_DWC3_DGCMD_POLL_MAX) {
			return false;
		}
	}

	return true;
}

/*
 * Flush one endpoint's TxFIFO via DGCMD.
 */
static void udc_dwc3_fifo_flush_tx(const struct device *const dev, const uint8_t fifo)
{
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	k_spinlock_key_t key;

	if (!udc_dwc3_dgcmd_wait_idle(base)) {
		LOG_ERR_RATELIMIT("a generic command stayed active over %u reads; TxFIFO "
				  "%u not flushed", UDC_DWC3_DGCMD_POLL_MAX, fifo);
		return;
	}

	/*
	 * The parameter and the command are ONE act and are locked as one.
	 * Unlocked, a force landing between these two writes overwrites DGCMDPAR
	 * with 0, so the flush below is issued against TxFIFO 0 instead of this
	 * endpoint's, and it is issued while the force's CMDACT is still set -
	 * writing over an active generic command, which the databook does not
	 * define.
	 */
	key = k_spin_lock(&priv->dgcmd_lock);

	sys_write32(UDC_DWC3_DGCMD_FIFOFLUSH_TX |
		    FIELD_PREP(UDC_DWC3_DGCMD_FIFOFLUSH_NUM_MASK, fifo),
		    base + UDC_DWC3_DGCMDPAR);
	sys_write32(UDC_DWC3_DGCMD_FIFOFLUSHONE | UDC_DWC3_DGCMD_ACT,
		    base + UDC_DWC3_DGCMD);

	k_spin_unlock(&priv->dgcmd_lock, key);

	/*
	 * Wait for it. The next control stage is armed immediately after this
	 * returns, and arming it while the flush is still in flight would race the
	 * very bytes being discarded.
	 */
	if (!udc_dwc3_dgcmd_wait_idle(base)) {
		LOG_ERR_RATELIMIT("TxFIFO %u flush did not complete in %u reads", fifo,
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
/*
 * How many times a non-control endpoint may re-issue a Start Transfer that came
 * back with a failing status before the endpoint is disabled instead.
 */

/* How many control-stage mismatches to name before going quiet. */
#define UDC_DWC3_CTRL_DESYNC_LOG_FIRST				12u

/* How many non-SETUP watchdog fires to describe before going quiet. */
#define UDC_DWC3_CTRL_WD_DUMP_FIRST				8u

/*
 * Return the control machine to step 1 without disturbing data endpoints.
 */
static void udc_dwc3_ctrl_resync(const struct device *const dev,
				 const char *const why)
{
	struct udc_dwc3_data *const priv = udc_get_private(dev);

	/*
	 * REPORT ONLY. These are the 4.4.x cases the models describe but do not
	 * ask software to stall for - step 3's "data stage on a request that
	 * declared wLength 0", step 3a's wrong direction, step 5a's exact-
	 * multiple zero-length OUT, and an undefined stage encoding.
	 */
	priv->ctrl_desync++;

	if (priv->ctrl_desync <= UDC_DWC3_CTRL_DESYNC_LOG_FIRST) {
		LOG_WRN("control stage mismatch #%u (reported, not acted on): %s",
			priv->ctrl_desync, why);
	}
}


/*
 * Check a control XferNotReady against the stage the programming model says is
 * current, and recover if the host is somewhere else.
 */
/*
 * 4.4.1/4.4.2: "issue Set Stall on EP0 and go back to Step 1".
 */
static void udc_dwc3_ctrl_stall(const struct device *const dev, const char *const why)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_data *const priv = udc_get_private(dev);

	priv->ctrl_stall_issued++;

	if (priv->ctrl_stall_issued <= UDC_DWC3_CTRL_DESYNC_LOG_FIRST) {
		LOG_WRN("control Set Stall #%u: %s", priv->ctrl_stall_issued, why);
	}

	udc_dwc3_depcmd_set_stall(dev, &cfg->ep_data_out[0]);

	/* Back to Step 1, through the one primitive that knows the whole set. */
	udc_dwc3_ctrl_reset_to_step1(dev, false);
	udc_dwc3_ctrl_next(dev);
}

/*
 * Validate an XferNotReady against the current control state.
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
	 * Stall.
	 */
	if (priv->ctrl_state == UDC_DWC3_CTRL_IDLE ||
	    priv->ctrl_state == UDC_DWC3_CTRL_STATUS_ARMED) {
		if (priv->ctrl_state == UDC_DWC3_CTRL_STATUS_ARMED) {
			udc_dwc3_ctrl_resync(dev,
				"late or duplicated XferNotReady for a request whose "
				"status stage is already armed");
			return false;
		}

		/*
		 * 4.4.1/4.4.2 step 2: an XferNotReady for a data or status stage
		 * arrived while no request is in flight, so it belongs to a
		 * transfer that is already over. Set Stall and go back to Step 1.
		 */
		udc_dwc3_ctrl_stall(dev,
			"XferNotReady for a stage of a request whose SETUP has not "
			"retired (step 2)");
		return false;
	}

	/*
	 * A status request here is legitimate and needs no further checking.
	 * Only the encoding the databook actually defines counts as one:
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
	 * (either direction), issue Set Stall on EP0 and go back to Step 1.
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
	 * stage it has already started, then issue Set Stall.
	 */
	if (is_in != wants_in) {
		udc_dwc3_ctrl_resync(dev,
			"host started the data stage in the direction opposite to "
			"bmRequestType");
		return false;
	}

	/*
	 * 4.4.2 step 5, a data XferNotReady arriving after the data stage
	 * already retired.
	 */
	if (priv->ctrl_state == UDC_DWC3_CTRL_DATA_DONE) {
		const uint16_t wlen = sys_le16_to_cpu(setup->wLength);
		const uint16_t mps = USB_MPS_EP_SIZE(cfg->ep_data_out[0].cfg.mps);

		if (mps != 0U && (wlen % mps) == 0U) {
			udc_dwc3_ctrl_resync(dev,
				"host is ending an exact-multiple data stage with a "
				"zero-length OUT packet, which needs a receive buffer "
				"the stack has already reclaimed");
		} else {
			/*
			 * 4.4.2 step 5b: the data stage has retired and the host is
			 * still moving data, so it is exceeding the wLength it
			 * declared. Set Stall and go back to Step 1.
			 */
			udc_dwc3_ctrl_stall(dev,
				"host is moving more data than the wLength it "
				"declared (step 5b)");
		}
		return false;
	}

	return true;
}

/*
 * XferNotReady on EP0-IN.
 */
static void udc_dwc3_on_xfer_not_ready_in(const struct device *const dev, const uint32_t evt)
{
	struct udc_dwc3_data *const priv = udc_get_private(dev);

	/*
	 * ALWAYS ARM THE NEXT STAGE. Detection must never suppress it.
	 * udc_dwc3_ctrl_resync() was made report-only after it destroyed cold
	 * boot, but the job was only half done: udc_dwc3_ctrl_xnr_check() still
	 * returns false at five sites and this function still returned on it,
	 * skipping udc_dwc3_ctrl_next() - the one call that arms the stage the
	 * host is asking for.
	 */
	if ((evt & UDC_DWC3_DEPEVT_STATUS_CONTROL_MASK) ==
	    UDC_DWC3_DEPEVT_STATUS_CONTROL_SETUP) {
		/*
		 * Rate limited for the same reason as every other report on this
		 * path:
		 */
		LOG_ERR_RATELIMIT("Invalid event (SETUP IN not possible)");
	} else {
		/*
		 * Record the status request BEFORE the check. The check can
		 * decline the event as a spec error case, but the host has still
		 * asked for the status stage, and 4.4.1 step 4 / 4.4.2 step 7
		 * arm it on that asking.
		 */
		if ((evt & UDC_DWC3_DEPEVT_STATUS_CONTROL_MASK) ==
		    UDC_DWC3_DEPEVT_STATUS_CONTROL_STATUS) {
			/*
			 * Only advance, never restart: an XferNotReady(Status)
			 * arriving in IDLE belongs to a transfer that is already
			 * over and is caught as step 2 by the check below.
			 */
			if (priv->ctrl_state == UDC_DWC3_CTRL_SETUP_DONE ||
			    priv->ctrl_state == UDC_DWC3_CTRL_DATA_DONE) {
				udc_dwc3_ctrl_state_set(dev, UDC_DWC3_CTRL_STATUS_READY);
			}
		}

		(void)udc_dwc3_ctrl_xnr_check(dev, evt, true);
	}

	udc_dwc3_ctrl_next(dev);
}

/*
 * XferNotReady on EP0-OUT.
 */
static void udc_dwc3_on_xfer_not_ready_out(const struct device *const dev, const uint32_t evt)
{
	struct udc_dwc3_data *const priv = udc_get_private(dev);

	/*
	 * ALWAYS ARM THE NEXT STAGE. Detection must never suppress it.
	 * udc_dwc3_ctrl_resync() was made report-only after it destroyed cold
	 * boot, but the job was only half done: udc_dwc3_ctrl_xnr_check() still
	 * returns false at five sites and this function still returned on it,
	 * skipping udc_dwc3_ctrl_next() - the one call that arms the stage the
	 * host is asking for.
	 */
	if ((evt & UDC_DWC3_DEPEVT_STATUS_CONTROL_MASK) ==
	    UDC_DWC3_DEPEVT_STATUS_CONTROL_SETUP) {
		LOG_ERR_RATELIMIT(
			"Invalid event (SETUP OUT not expected to have an event)");
	} else {
		/*
		 * Record the status request BEFORE the check. The check can
		 * decline the event as a spec error case, but the host has still
		 * asked for the status stage, and 4.4.1 step 4 / 4.4.2 step 7
		 * arm it on that asking.
		 */
		if ((evt & UDC_DWC3_DEPEVT_STATUS_CONTROL_MASK) ==
		    UDC_DWC3_DEPEVT_STATUS_CONTROL_STATUS) {
			/*
			 * Only advance, never restart: an XferNotReady(Status)
			 * arriving in IDLE belongs to a transfer that is already
			 * over and is caught as step 2 by the check below.
			 */
			if (priv->ctrl_state == UDC_DWC3_CTRL_SETUP_DONE ||
			    priv->ctrl_state == UDC_DWC3_CTRL_DATA_DONE) {
				udc_dwc3_ctrl_state_set(dev, UDC_DWC3_CTRL_STATUS_READY);
			}
		}

		(void)udc_dwc3_ctrl_xnr_check(dev, evt, false);
	}

	udc_dwc3_ctrl_next(dev);
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
		LOG_ERR_RATELIMIT("Invalid TRB type: 0x%08lx",
				  (trb->status & UDC_DWC3_TRB_STATUS_TRBSTS_MASK));
		break;
	}
}


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

		ret = udc_dwc3_pop_trb(ep_data, &buf, &trb);
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

		udc_dwc3_on_xfer_done(&trb);

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
		drained++;
	}

	return drained;
}

/*
 * True if an event word is the given DEPEVT type on any endpoint.
 */
static inline bool udc_dwc3_evt_is_depevt(const uint32_t evt_type,
					 const uint32_t depevt_ep0);

/*
 * Transfer completion on a non-control endpoint.
 */
static void udc_dwc3_on_xfer_done_nonctrl(const struct device *const dev, const uint32_t evt)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	const int epn = FIELD_GET(UDC_DWC3_DEPEVT_EPN_MASK, evt);
	struct udc_dwc3_ep_data *ep_data;

	if (!_EPN_IS_VALID(cfg, epn)) {
		LOG_ERR_RATELIMIT("event 0x%08x names physical endpoint %d, which "
				  "this controller does not have (%u IN, %u OUT) - "
				  "discarded",
				  evt, epn, cfg->num_in_eps, cfg->num_out_eps);
		return;
	}

	ep_data = _EP_DATA_FROM_EPN(cfg, epn);

	(void)udc_dwc3_drain_completed(dev, ep_data);

	/*
	 * XFERCOMPLETE AND XFERINPROGRESS MEAN OPPOSITE THINGS ABOUT THE
	 * TRANSFER RESOURCE, and until now this handler treated them as one.
	 * Databook 3.2.2.2:
	 */
	if (udc_dwc3_evt_is_depevt(evt & UDC_DWC3_EVT_MASK,
				   UDC_DWC3_DEPEVT_XFERCOMPLETE(0))) {
		/*
		 * ONLY IF THE ENDPOINT REALLY HAS NOTHING LEFT. A skipped slot
		 * can be filled by its late write afterwards, so a well-formed
		 * event can be read one ring wrap later, belonging to a transfer
		 * that ended long ago. It is indistinguishable from a fresh one
		 * by inspection - there is no epoch or sequence field in the
		 * event word to tell them apart.
		 */
		if (udc_dwc3_ep_ring_outstanding(ep_data)) {
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

#define _NORMAL_EP(n, fn) fn(n + 2)

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
 * Park every armed buffer on an endpoint back on its requeue FIFO and reset the
 * TRB ring to the empty state.
 */
static void udc_dwc3_ep_ring_release(struct udc_dwc3_ep_data *const ep_data)
{
	/* TRB_NUM - 1 is deliberate: the last TRB is the LINK descriptor
	 * (see udc_dwc3_trb_nonctrl_init), which stays armed to keep the ring
	 * intact; only the payload slots are drained and cleared.
	 */
	const int slots = CONFIG_UDC_DWC3_TRB_NUM - 1;
	struct net_buf *buf;

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
}

/*
 * Endpoint Command Complete.
 */
static void udc_dwc3_on_ep_cmd_cmplt(const struct device *const dev, const uint32_t evt)
{
	bool resume_queued = false;
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const int epn = FIELD_GET(UDC_DWC3_DEPEVT_EPN_MASK, evt);
	struct udc_dwc3_ep_data *ep_data;

	if (!_EPN_IS_VALID(cfg, epn)) {
		LOG_ERR_RATELIMIT("event 0x%08x names physical endpoint %d, which "
				  "this controller does not have (%u IN, %u OUT) - "
				  "discarded",
				  evt, epn, cfg->num_in_eps, cfg->num_out_eps);
		return;
	}

	ep_data = _EP_DATA_FROM_EPN(cfg, epn);

	/*
	 * A Start Transfer completion carries the transfer resource index this
	 * driver needs for every later Update and End Transfer on this endpoint.
	 */
	if (FIELD_GET(UDC_DWC3_DEPEVT_CMDTYP_MASK, evt) ==
	    FIELD_GET(UDC_DWC3_DEPCMD_CMDTYP_MASK, UDC_DWC3_DEPCMD_DEPSTRTXFER)) {
		if ((evt & UDC_DWC3_DEPEVT_CMDSTATUS_MASK) != UDC_DWC3_DEPCMD_STATUS_OK) {
			bool already_reported = false;

			priv->ctrl_start_fail++;

			/*
			 * Mark it reported. depcmd()'s one-command-late CMDERR
			 * line reads this bit to avoid saying the same failure
			 * twice, and the only place that used to set it was the
			 * result-waiting path that Start Transfer no longer
			 * takes.
			 */
			if (epn < UDC_DWC3_MAX_EPN) {
				already_reported =
					(priv->start_fail_reported & BIT(epn)) != 0;
				priv->start_fail_reported |= BIT(epn);
				priv->depcmd_reported |= BIT(epn);
			}

			if (!already_reported) {
				LOG_ERR("EP%02x Start Transfer reported status 0x%x in "
					"its Command Complete, keeping transfer resource "
					"index 0x%x%s (%u so far)", ep_data->cfg.addr,
					(unsigned int)FIELD_GET(UDC_DWC3_DEPEVT_CMDSTATUS_MASK,
								evt),
					ep_data->xferrscidx,
					ep_data->xferrscidx == UDC_DWC3_XFERRSCIDX_INVALID
					? " (never established)" : "",
					priv->ctrl_start_fail);
			}

			/*
			 * RELEASE THE CLAIM. This is the whole point of handling the
			 * failure here rather than only logging it.
			 */
			udc_ep_set_busy(&ep_data->cfg, false);

			/*
			 * Re-offer the stage. This handler already runs under the UDC
			 * mutex - udc_dwc3_handle_event() holds it for the whole
			 * dispatch - so this is the same context ep_enqueue arms from.
			 */
			if (USB_EP_GET_IDX(ep_data->cfg.addr) == 0) {
				udc_dwc3_ctrl_next(dev);
				return;
			}

			/*
			 * A NON-CONTROL endpoint cannot recover the way a
			 * control one does.
			 */
			/*
			 * NO RETRY HERE EITHER, for the reason given at the
			 * other Start Transfer retry:
			 */

			/*
			 * Out of retries, or the re-init could not even issue.
			 * Take the endpoint down the same way
			 * udc_dwc3_ep_disable() does, short of issuing an End
			 * Transfer against a resource that does not exist:
			 */
			sys_clear_bits(DEVICE_MMIO_NAMED_GET(dev, base) +
					       UDC_DWC3_DALEPENA,
				       UDC_DWC3_DALEPENA_USBACTEP(ep_data->epn));

			LOG_ERR("EP%02x transfer disabled: no transfer resource was ever "
				"assigned; parking queued buffers for re-enable",
				ep_data->cfg.addr);

			udc_dwc3_ep_ring_release(ep_data);

			/*
			 * Bring the rest of the endpoint state in line with the
			 * driver state below.
			 */
			udc_dwc3_ep_state_reset(ep_data);
			udc_ep_set_busy(&ep_data->cfg, false);

			/*
			 * The stack still records this endpoint as enabled,
			 * because the enable that posted the Start Transfer
			 * returned success before the command could fail.
			 */
			udc_submit_event(dev, UDC_EVT_ERROR, -EIO);

			return;
		}

		/* A resource was assigned, so the retry budget is spent on nothing. */

		udc_dwc3_adopt_xferrscidx_evt(dev, ep_data,
			FIELD_GET(UDC_DWC3_DEPEVT_XFERRSCIDX_MASK, evt));
		LOG_DBG("EP%02x transfer resource index taken from the event",
			ep_data->cfg.addr);

		/*
		 * The requeue loop in udc_dwc3_ep_resume() runs while the UDC
		 * mutex is held, so its Update Transfer commands are all refused
		 * for want of this index before this Command Complete has been
		 * drained.
		 */
		if (USB_EP_GET_IDX(ep_data->cfg.addr) > 0 &&
		    (ep_data->head != ep_data->tail || ep_data->full)) {
			if (!udc_dwc3_depcmd_update_xfer(dev, ep_data)) {
				LOG_ERR("EP%02x start complete but Update Transfer refused "
					"with armed ring", ep_data->cfg.addr);
			}
		}
		return;
	}

	if (!udc_dwc3_ep_is_ending(ep_data)) {
		/*
		 * NOT OURS - DISCARD IT.
		 *
		 * CMDIOC is requested on Start Transfer and End Transfer only, so
		 * reaching here means an End Transfer completion arrived for an
		 * endpoint that is not ending. The endpoint has been torn down and
		 * re-established since that command was posted - an alt-setting
		 * switch is a disable/enable pair and needs no lost event at all -
		 * so the completion describes a previous incarnation.
		 *
		 * Falling through reset the NEW transfer to IDLE and discarded the
		 * index the controller had just assigned it: Update Transfer is
		 * then refused for want of a resource index, and the recovery
		 * machine answers by taking a second transfer resource for the
		 * same endpoint.
		 */
		priv->ep_cmd_cmplt_stale++;
		LOG_WRN_RATELIMIT("EpCmdCmplt on EP%02x with no End Transfer "
				  "outstanding (%s): discarded",
				  ep_data->cfg.addr,
				  udc_dwc3_ep_state_name(ep_data->xfer_state));
		return;
	}

	/*
	 * The controller has let the transfer go, so the resource is back and
	 * the endpoint is idle.
	 */
	resume_queued = (ep_data->xfer_state == UDC_DWC3_EP_ENDING_RESUME);
	udc_dwc3_ep_state_reset(ep_data);

	LOG_DBG("EpCmdCmplt: DMA stopped for EP%02x", ep_data->cfg.addr);

	/*
	 * THE RE-ARM IS NOT DONE HERE. This used to carry a second, complete
	 * recovery - ring_release, resume, Start Transfer - gated on
	 * wedge_recover_pending, with its own counters and its own log lines. It
	 * has moved into udc_dwc3_recover_step(), which reaches the same place
	 * through RECLAIM -> RESTART and bounds the wait for this very
	 * completion.
	 */

	/*
	 * Second half of a non-control resume that udc_dwc3_ep_resume()
	 * postponed because this End Transfer was still concluding.
	 */
	if (resume_queued) {
		int ret;

		LOG_DBG("running deferred resume for EP%02x", ep_data->cfg.addr);

		ret = udc_dwc3_ep_resume(dev, ep_data, ep_data->resume_modify);
		if (ret != 0) {
			LOG_ERR("deferred resume failed on EP%02x: %d",
				ep_data->cfg.addr, ret);
			udc_submit_event(dev, UDC_EVT_ERROR, ret);
		}
	} else if (USB_EP_GET_IDX(ep_data->cfg.addr) > 0 && ep_data->cfg.stat.enabled) {
		/*
		 * No resume was postponed, but udc_dwc3_ep_worker() may have
		 * stopped while this endpoint was ENDING, and nothing else would
		 * wake it - udc_dwc3_ep_enqueue() only submits the work when a
		 * new buffer arrives.
		 */
		k_work_submit_to_queue(udc_get_work_q(), &ep_data->work);
	} else if (USB_EP_GET_IDX(ep_data->cfg.addr) == 0) {
		/*
		 * Control counterpart of the same wake-up. udc_dwc3_ctrl_try() declines
		 * to arm while the endpoint is ENDING, and the buffer that was refused
		 * is still queued with nothing scheduled to look at it again - the
		 * control path has no work queue of its own.
		 */
		udc_dwc3_ctrl_next(dev);
	}
}

/*
 * Report a USB/Link State Change event without letting the hardware set the log
 * rate.
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

/*
 * True when evt_type is the given DEPEVT on ANY endpoint.  The endpoint number
 * occupies bits 5:1 of the event type, so masking it off compares the event kind
 * alone - see UDC_DWC3_DEPEVT_XFERINPROGRESS(epn).
 */
static inline bool udc_dwc3_evt_is_depevt(const uint32_t evt_type, const uint32_t depevt_ep0)
{
	return (evt_type & ~GENMASK(5, 1)) == (depevt_ep0 & ~GENMASK(5, 1));
}

/*
 * Dispatch one event word. Runs under the UDC mutex.
 */
static void udc_dwc3_handle_event(const struct device *const dev, const uint32_t evt)
{
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	const uint32_t dsts = sys_read32(base + UDC_DWC3_DSTS);

	/* Both banners are logged OUTSIDE the mutex, deliberately. */
	const uint32_t evt_type = evt & UDC_DWC3_EVT_MASK;
	const bool is_link_evt = evt_type == UDC_DWC3_DEVT_ULSTCHNG;
	/*
	 * Events the CONTROLLER can produce faster than this console can print
	 * them.
	 */
	const bool is_ovfl_evt = evt_type == UDC_DWC3_DEVT_EVNTOVERFLOW;
	/* Generic command completions are silent for the same reason. */
	const bool is_cmdcmplt_evt = evt_type == UDC_DWC3_DEVT_CMDCMPLT;
	/*
	 * The events every healthy transfer generates. Naming each one costs a
	 * synchronous uart_poll_out in the drain thread; a failing endpoint
	 * command is reported with its status by udc_dwc3_on_ep_cmd_cmplt().
	 */
	const bool is_xfer_evt =
		udc_dwc3_evt_is_depevt(evt_type, UDC_DWC3_DEPEVT_XFERCOMPLETE(0)) ||
		udc_dwc3_evt_is_depevt(evt_type, UDC_DWC3_DEPEVT_XFERINPROGRESS(0)) ||
		udc_dwc3_evt_is_depevt(evt_type, UDC_DWC3_DEPEVT_XFERNOTREADY(0)) ||
		udc_dwc3_evt_is_depevt(evt_type, UDC_DWC3_DEPEVT_EPCMDCMPLT(0));
	const bool is_quiet_evt = is_link_evt || is_ovfl_evt || is_cmdcmplt_evt ||
				  is_xfer_evt;

	if (is_link_evt) {
		udc_dwc3_log_link_event(dev, evt, dsts);
	} else if (!is_quiet_evt) {
		LOG_INF("%s", udc_dwc3_get_event_name(evt, dsts));
	}

	/*
	 * Published for udc_dwc3_heartbeat_worker(), which runs on another
	 * thread.
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
	 * Both completion events retire TRBs and both mean success. Which one
	 * the controller raises depends only on the TRB control bits, per Table
	 * 4-8:
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
		 * The link is gone, so any Endpoint Command Complete still
		 * outstanding is not coming.
		 */
		udc_dwc3_drop_xfer_state(dev, "disconnect");
		break;
	/* XferNotReady on a NON-CONTROL endpoint: ignore it. The databook: */
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
		 * Erratic error. On UTMI+ this means phy_rxvalid/phy_rxactive
		 * stayed asserted for at least 2 ms;
		 */
		/*
		 * Rate-limited because nothing here clears the fault: the
		 * comment above says the link does not come back on its own, so
		 * the controller is free to raise this again on every pass.
		 */
		LOG_ERR_RATELIMIT("DEVT_ERRTICERR: PHY erratic error - the link is "
			"suspended and needs a disconnect/reconnect to recover");
		udc_submit_event(dev, UDC_EVT_ERROR, -EIO);
		break;
	case UDC_DWC3_DEVT_EVNTOVERFLOW:
		/*
		 * The only line for this event now - the generic banner and
		 * "end" are suppressed above.
		 */
		LOG_ERR_RATELIMIT("evt ring ovfl");
		break;
	default:
		/* Skip the event, do not assume it cannot happen. */
		LOG_ERR_RATELIMIT("unknown event: 0x%x (%u out of %u)",
				  evt,
				  priv->evt_gc_last,
				  CONFIG_UDC_DWC3_EVENTS_NUM);
		break;
	}

	udc_unlock_internal(dev);

	priv->dispatch_evt = 0U;

	/* Outside the lock - see the note above the opening banner. */
	if (!is_quiet_evt) {
		LOG_DBG("end");
	}
}

/*
 * Liveness, checked from the system work queue.
 */
/*
 * ISR context, so this is limited to what is legal and cheap there: an MMIO
 * read, a subtraction, and k_work_submit_to_queue() - which is explicitly
 * ISR-safe. No mutex, no logging (LOG_MODE_MINIMAL busy-waits on the console
 * UART and would hold interrupts off for milliseconds), no udc_dwc3_recover().
 * Anything that needs those runs in udc_dwc3_heartbeat_worker() instead.
 */
static const char *udc_dwc3_drain_state_name(const uint32_t state)
{
	switch (state) {
	case UDC_DWC3_DRAIN_IDLE:	return "idle";
	case UDC_DWC3_DRAIN_RUNNING:	return "running";
	case UDC_DWC3_DRAIN_WAITING:	return "waiting";
	case UDC_DWC3_DRAIN_NUDGE:	return "nudge";
	case UDC_DWC3_DRAIN_PARTIAL:	return "partial";
	default:			return "?";
	}
}

static void udc_dwc3_drain_helper(const struct device *const dev);

/*
 * Heartbeat timer callback. Kicks the drain and submits the heartbeat worker.
 */
static void udc_dwc3_heartbeat_expiry(struct k_timer *const timer)
{
	struct udc_dwc3_data *const priv =
		CONTAINER_OF(timer, struct udc_dwc3_data, heartbeat_timer);
	const struct device *const dev = priv->dev;

	udc_dwc3_drain_helper(dev);

	priv->hb_expiries++;

	/* Stamped at submit, read at worker entry: the queue wait, measured. */
	if (priv->hb_submit_t == 0U) {
		priv->hb_submit_t = k_cycle_get_32();
	}

	/*
	 * 0 means "already pending": the previous beat has not been dispatched
	 * yet, so this one is dropped rather than run late.
	 */
	if (k_work_submit_to_queue(udc_get_work_q(),
				   &priv->heartbeat_work) == 0) {
		priv->hb_coalesced++;
	}
}

#ifdef STALL_DIAG_LOG
/*
 * Space left in ONE endpoint's TxFIFO.
 */
static uint32_t udc_dwc3_txfifo_space(const struct device *const dev,
				      const struct udc_dwc3_ep_data *const e)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint32_t sel;

	if (!USB_EP_DIR_IS_IN(e->cfg.addr)) {
		return 0U;
	}

	sel = UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_TXFIFO |
	      FIELD_PREP(UDC_DWC3_GDBGFIFOSPACE_QUEUENUM_MASK,
			 (uint32_t)(e->cfg.addr & 0x7fU));
	sys_write32(sel, base + UDC_DWC3_GDBGFIFOSPACE);

	return FIELD_GET(UDC_DWC3_GDBGFIFOSPACE_AVAILABLE_MASK,
			 sys_read32(base + UDC_DWC3_GDBGFIFOSPACE));
}

/*
 * One hardware state dump, for the RTL side, on either shape of failure.
 */
static void udc_dwc3_epstate_dump(const struct device *const dev, const char *const tag);

/*
 * Diagnostic dump for a stalled endpoint.
 */
static void udc_dwc3_stall_diag_dump(const struct device *const dev,
				     const char *const why)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

	/* Dump header, not a fault: severity belongs to the line that triggered
	 * this. It is reached from the benign control-idle path as well as from
	 * real wedges, and an E: here made the benign case read as a failure. */
	LOG_INF("=== STALL DIAG (%s) ===", why);
	const uint32_t gsts = sys_read32(base + UDC_DWC3_GSTS);

	/*
	 * The one register that can turn this from an observation into a
	 * hardware fault report. 1.2.13:
	 */
	LOG_INF("  BUS: GSTS=0x%08x BusErrAddrVld=%u "
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
	LOG_INF("  RING: GEVNTADR=0x%08x%08x SIZ=0x%08x CNT=0x%08x "
		"| driver buf=%p stalled slot %u at %p",
		sys_read32(base + UDC_DWC3_GEVNTADR_HI(0)),
		sys_read32(base + UDC_DWC3_GEVNTADR_LO(0)),
		sys_read32(base + UDC_DWC3_GEVNTSIZ(0)),
		priv->evt_gc_last,
		(void *)cfg->evt_buf, priv->evt_next,
		(void *)&cfg->evt_buf[priv->evt_next]);

	/* EVERY endpoint, not just EP0. */
	/*
	 * The RxFIFO, and why it is worth dumping. Device mode has exactly ONE
	 * receive FIFO for every OUT endpoint: the databook states "Since the
	 * device mode uses only one RXFIFO, there is no Device RXFIFO DMA
	 * Priority Register", and GRXFIFOSIZ0 "allocate[s] the receive buffer
	 * for all endpoints".
	 */
	{
		const uint32_t rxsz = sys_read32(base + UDC_DWC3_GRXFIFOSIZ(0));
		const uint32_t mdw = (sys_read32(base + UDC_DWC3_GHWPARAMS0) >> 8) & 0xFFU;

		{
		const uint32_t now = k_cycle_get_32();

		LOG_INF("  CTRLTRACE: setup_up n=%u %ums ago | enq n=%u %ums ago "
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

	LOG_INF("  RXFIFO: GRXFIFOSIZ0=0x%08x depth=%u start=%u mdwidth=%u "
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
			 * FREE SPACE in the shared RxFIFO, not bytes queued for
			 * this endpoint.
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

			LOG_INF("  EP%02x: busy=%u hwo=%u trbctl=%u ctrl=0x%08x "
				"sts=0x%08x head=%u tail=%u full=%u endxfer=%u "
				"depcmd=0x%08x last=0x%08x txfifo=%u rxfree=%u",
				e->cfg.addr,
				udc_ep_is_busy(&e->cfg) ? 1U : 0U,
				(c & UDC_DWC3_TRB_CTRL_HWO) ? 1U : 0U,
				(uint32_t)((c & UDC_DWC3_TRB_CTRL_TRBCTL_MASK) >> 4),
				c, t[e->tail].status,
				e->head, e->tail, e->full ? 1U : 0U,
				udc_dwc3_ep_is_ending(e) ? 1U : 0U,
				sys_read32(base + UDC_DWC3_DEPCMD(e->epn)),
				e->epn < UDC_DWC3_MAX_EPN ?
					priv->depcmd_last[e->epn] : 0U,
				udc_dwc3_txfifo_space(dev, e),
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
		LOG_INF("  RING[%02u]: 0x%08x 0x%08x 0x%08x 0x%08x",
			i, cfg->evt_buf[i], cfg->evt_buf[i + 1],
			cfg->evt_buf[i + 2], cfg->evt_buf[i + 3]);
	}

	udc_dwc3_core_state_dump(dev);

	/* For correlating with an ILA or bus trace. */
	LOG_INF("  AT: DSTS=0x%08x cycles=%u",
		sys_read32(base + UDC_DWC3_DSTS), k_cycle_get_32());

	/*
	 * Last, because it is the only part of this that issues commands, and
	 * everything passive above must be recorded before the core is touched.
	 */
	priv->ctrl_stall_captured = true;
	udc_dwc3_epstate_dump(dev, "CTRLSTALL");
}
#endif /* STALL_DIAG_LOG */

/*
 * Is the empty slot at evt_next provably lost rather than merely late?
 */
static bool udc_dwc3_evt_lookahead_lost(const struct device *const dev, const uint32_t gc)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	uint32_t owed = gc / sizeof(uint32_t);

	if (owed > (CONFIG_UDC_DWC3_EVENTS_NUM - 1u)) {
		owed = CONFIG_UDC_DWC3_EVENTS_NUM - 1u;
	}

	for (uint32_t j = 1u; j < owed; j++) {
		const uint32_t idx = (priv->evt_next + j) % CONFIG_UDC_DWC3_EVENTS_NUM;

		if (cfg->evt_buf[idx] != UDC_DWC3_EVT_CONSUMED_ENTRY_VALUE) {
			return true;
		}
	}

	return false;
}

/* Defined below; the heartbeat owns the decision, this performs it. */
static uint32_t udc_dwc3_evt_skip_dead_slot(const struct device *const dev,
					const uint32_t gc, const bool frozen,
					const uint32_t gaveup_ms);

/*
 * Capture DEPGETSTATE for every endpoint this driver drives.
 *
 * Databook 3.2.2.3: the 32 bits returned in DEPCMDPAR2 are "the current data
 * sequence number, flow control state, and control transfer state (for control
 * endpoints)". No field layout is published - the command is documented only for
 * hibernation - so a single value says nothing. What is worth capturing is the
 * DIFFERENCE between the same endpoint wedged and that endpoint working, which is
 * why this is called both when a wedge is declared and again when it clears.
 * When the bitfield decoding arrives, that pair says which bits were wrong.
 */
/*
 * The healthy half of the control-endpoint pair. Called when control traffic
 * moves again after a stall was captured, so the two dumps bracket the failure on
 * the same endpoints, seconds apart.
 */
static void udc_dwc3_ctrl_stall_cleared(const struct device *const dev)
{
	struct udc_dwc3_data *const priv = udc_get_private(dev);

	if (!priv->ctrl_stall_captured) {
		return;
	}

	priv->ctrl_stall_captured = false;
	LOG_ERR("control traffic resumed after a stall capture");
	udc_dwc3_epstate_dump(dev, "CTRLCLEARED");
}

/*
 * DEPGETSTATE for every endpoint this driver drives.
 */
static void udc_dwc3_epstate_dump(const struct device *const dev, const char *const tag)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	static const uint8_t epns[] = { 0U, 1U, 2U, 5U, 9U };
	static const char *const names[] = { "EP00", "EP80", "EP01", "EP82", "EP84" };


	for (uint32_t i = 0U; i < ARRAY_SIZE(epns); i++) {
		const uint32_t epn = epns[i];
		const bool dir_in = (epn & 1U) != 0U;
		const uint32_t log_ep = epn >> 1;
		struct udc_dwc3_ep_data *ep_data;
		uint32_t par2;

		if (dir_in ? (cfg->num_in_eps <= log_ep) : (cfg->num_out_eps <= log_ep)) {
			continue;
		}
		ep_data = dir_in ? &cfg->ep_data_in[log_ep] : &cfg->ep_data_out[log_ep];

		/*
		 * Twice. The first DEPGETSTATE on an endpoint can return a stale
		 * PAR2 - that is what produced a bogus 0x00000004 in the first
		 * capture of every earlier run.
		 */
		udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(epn), UDC_DWC3_DEPCMD_DEPGETSTATE);
		udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(epn), UDC_DWC3_DEPCMD_DEPGETSTATE);
		par2 = sys_read32(base + UDC_DWC3_DEPCMDPAR2(epn));

		if (ep_data->trb_buf == NULL) {
			LOG_INF("EPSTATE %-9s %s epn=%u par2=0x%08x depcmd=0x%08x "
				"(not configured)", tag, names[i], epn, par2,
				sys_read32(base + UDC_DWC3_DEPCMD(epn)));
			continue;
		}

		/*
		 * depcmd= is the DEPGETSTATE this function just issued twice,
		 * not the command that preceded the fault - reading the register
		 * here reports our own footprint.
		 */
		LOG_INF("EPSTATE %-9s %s epn=%u par2=0x%08x depcmd=0x%08x last=0x%08x "
			"txfifo=%u | busy=%u "
			"head=%u tail=%u n_retire=%u arm=%u trb ctrl=0x%08x sts=0x%08x "
			"rscidx=0x%x", tag, names[i], epn, par2,
			sys_read32(base + UDC_DWC3_DEPCMD(epn)),
			epn < UDC_DWC3_MAX_EPN ? priv->depcmd_last[epn] : 0U,
			udc_dwc3_txfifo_space(dev, ep_data),
			udc_ep_is_busy(&ep_data->cfg) ? 1U : 0U,
			ep_data->head, ep_data->tail, ep_data->n_retire,
			ep_data->armed_len[ep_data->tail],
			ep_data->trb_buf[ep_data->tail].ctrl,
			ep_data->trb_buf[ep_data->tail].status,
			ep_data->xferrscidx);
	}

}

/*
 * Everything the EP wedge needs and the per-endpoint dump does not carry.
 */
static void udc_dwc3_wedge_core_dump(const struct device *const dev,
				     const struct udc_dwc3_ep_data *const ep_data)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	const uint32_t gsts = sys_read32(base + UDC_DWC3_GSTS);
	const uint32_t dsts = sys_read32(base + UDC_DWC3_DSTS);

	LOG_INF("  WBUS: GSTS=0x%08x BusErrAddrVld=%u GBUSERRADDR=0x%08x%08x",
		gsts, (gsts & UDC_DWC3_GSTS_BUSERRADDRVLD) ? 1U : 0U,
		sys_read32(base + UDC_DWC3_GBUSERRADDR_HI),
		sys_read32(base + UDC_DWC3_GBUSERRADDR_LO));

	/*
	 * GEVNTCOUNT is the one number that separates "the controller posted a
	 * completion this driver has not drained" from "the controller posted
	 * nothing". Without it the two are indistinguishable in the log.
	 */
	LOG_INF("  WRING: GEVNTADR=0x%08x%08x SIZ=0x%08x CNT=0x%08x "
		"| driver buf=%p next slot %u",
		sys_read32(base + UDC_DWC3_GEVNTADR_HI(0)),
		sys_read32(base + UDC_DWC3_GEVNTADR_LO(0)),
		sys_read32(base + UDC_DWC3_GEVNTSIZ(0)),
		priv->evt_gc_last,
		(void *)cfg->evt_buf, priv->evt_next);

	LOG_INF("  WEVT: handled %u late %u (worst %u polls/%u us) gaveup %u "
		"(worst empty %u us) isr %u runs %u rearm %u kick %u skip %u "
		"zero %u missed %u/%u midzero %u gc_hwm %u B gc0max %u B multi %u "
		"| drain %s slot %u attempts %u",
		priv->evt_handled, priv->evt_late,
		priv->evt_late_polls_max, priv->evt_late_us_max,
		priv->evt_gaveup, priv->evt_gaveup_us_max,
		priv->evt_isr, priv->evt_worker_runs, priv->evt_rearm,
		priv->evt_kick, priv->evt_skipped,
		priv->evt_zero, priv->evt_missed, priv->evt_missed_frozen,
		priv->evt_midzero, priv->evt_gevntcount_hwm, priv->evt_gaveup_gc0_max,
		priv->evt_gaveup_multi,
		udc_dwc3_drain_state_name(priv->drain.state),
		priv->drain.slot, priv->drain.attempts);

	LOG_INF("  WDEV: DSTS=0x%08x (%s) DCTL=0x%08x DCFG=0x%08x DALEPENA=0x%08x",
		dsts, udc_dwc3_get_devt_ulstchng_name(dsts),
		sys_read32(base + UDC_DWC3_DCTL),
		sys_read32(base + UDC_DWC3_DCFG),
		sys_read32(base + UDC_DWC3_DALEPENA));

	/*
	 * The whole ring, not just the tail. A controller that skipped one
	 * descriptor and moved on looks identical at the tail to one that stopped
	 * dead; the slots ahead of it are what tell them apart.
	 */
	if (ep_data->trb_buf != NULL) {
		for (uint32_t i = 0; i < CONFIG_UDC_DWC3_TRB_NUM; i++) {
			LOG_INF("  WTRB: EP%02x s%u addr=0x%08x%08x sts=0x%08x "
				"ctrl=0x%08x armed=%u%s",
				ep_data->cfg.addr, i,
				ep_data->trb_buf[i].addr_hi,
				ep_data->trb_buf[i].addr_lo,
				ep_data->trb_buf[i].status,
				ep_data->trb_buf[i].ctrl,
				ep_data->armed_len[i],
				i == ep_data->tail ? " <-TAIL" :
					(i == ep_data->head ? " <-HEAD" : ""));
		}
	}

	udc_dwc3_core_state_dump(dev);
}

/* ------------------------------------------------------------------------- *
 * THE recovery function. One call site: udc_dwc3_heartbeat_worker().
 * ------------------------------------------------------------------------- */

static const char *udc_dwc3_recov_state_name(const uint8_t st)
{
	switch (st) {
	case UDC_DWC3_RECOV_IDLE:	return "idle";
	case UDC_DWC3_RECOV_SUSPECT:	return "suspect";
	case UDC_DWC3_RECOV_RESYNC:	return "resync";
	case UDC_DWC3_RECOV_RECLAIM:	return "reclaim";
	case UDC_DWC3_RECOV_RESTART:	return "restart";
	case UDC_DWC3_RECOV_STALL:	return "stall";
	case UDC_DWC3_RECOV_FAILED:	return "failed";
	default:			return "?";
	}
}

/*
 * Name of a recovery reason, for logging.
 */
static const char *udc_dwc3_recov_reason_name(const uint8_t r)
{
	switch (r) {
	case UDC_DWC3_RECOV_R_UNPOPPED:		return "completion lost (HWO=0, not popped)";
	case UDC_DWC3_RECOV_R_UNFETCHED:	return "descriptor never fetched (HWO=1)";
	case UDC_DWC3_RECOV_R_NO_RESOURCE:	return "no transfer resource (Start Transfer completion lost)";
	case UDC_DWC3_RECOV_R_ENDXFER_LOST:	return "End Transfer completion never arrived";
	case UDC_DWC3_RECOV_R_NO_SETUP:		return "control stage unarmed while the host asks";
	case UDC_DWC3_RECOV_R_SLOT_LOST:	return "event announced but never written";
	default:				return "none";
	}
}

/*
 * True while the state machine is mid-episode on this endpoint and the older
 * control-path mechanisms should leave it to finish.
 */
static bool udc_dwc3_recov_owns(const struct udc_dwc3_ep_data *const ep_data)
{
	return ep_data->recov.state != UDC_DWC3_RECOV_IDLE &&
	       ep_data->recov.state != UDC_DWC3_RECOV_FAILED;
}

/*
 * How long one piece of evidence must persist before it is acted on.
 *
 * One function, so the answer to "why did it wait that long" is in one place.
 */
static uint32_t udc_dwc3_recov_dwell_ms(const struct udc_dwc3_ep_data *const ep_data,
					const uint8_t reason)
{
	switch (reason) {
	case UDC_DWC3_RECOV_R_UNFETCHED:
		/*
		 * A descriptor the controller has not sent yet. Whether that is a
		 * fault depends on when the host next polls, so this waits far longer
		 * than anything else - except on EP0, which is on the host's clock.
		 */
		return (USB_EP_GET_IDX(ep_data->cfg.addr) == 0U)
			? UDC_DWC3_RECOV_CTRL_UNFETCHED_MS
			: UDC_DWC3_RECOV_UNFETCHED_MS;

	case UDC_DWC3_RECOV_R_NO_RESOURCE:
	case UDC_DWC3_RECOV_R_ENDXFER_LOST:
		/*
		 * Both of these say an EpCmdCmplt that was due has not arrived -
		 * one for a Start Transfer, one for an End Transfer.
		 */
		return UDC_DWC3_RECOV_CMDCMPLT_MS;

	default:
		/*
		 * UNPOPPED and NO_SETUP: the work is already finished, or the host is
		 * actively asking. Nothing is gained by waiting longer.
		 */
		return UDC_DWC3_RECOV_SUSPECT_MS;
	}
}

/*
 * THE ONLY WAY INTO FAILED.
 */
static void udc_dwc3_recov_fail(const struct device *const dev,
				struct udc_dwc3_ep_data *const ep_data,
				const char *const why)
{
	struct udc_dwc3_recov *const rc = &ep_data->recov;

	rc->state = UDC_DWC3_RECOV_FAILED;
	rc->n_failed++;

	/*
	 * Report the first few episodes in full, then go quiet. The counter in the
	 * "-> idle after a retire" line carries the rest.
	 */
	if (rc->n_failed > UDC_DWC3_RECOV_REPORT_MAX) {
		return;
	}

	LOG_ERR("EP%02x: recovery GAVE UP after %u action(s) - %s: %s (episode %u)",
		ep_data->cfg.addr, rc->attempts,
		udc_dwc3_recov_reason_name(rc->reason), why, rc->n_failed);
	udc_dwc3_wedge_core_dump(dev, ep_data);
}

/*
 * Resolve a STARTING endpoint by asking the controller directly.
 */
static void udc_dwc3_ep_resolve_starting(const struct device *const dev,
					 struct udc_dwc3_ep_data *const ep_data)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint32_t reg;

	if (ep_data->xfer_state != UDC_DWC3_EP_STARTING) {
		return;
	}

	reg = sys_read32(base + UDC_DWC3_DEPCMD(ep_data->epn));

	if ((reg & UDC_DWC3_DEPCMD_CMDACT) != 0U) {
		return;		/* still running; nothing to decide yet */
	}

	if ((reg & UDC_DWC3_DEPCMD_STATUS_MASK) != UDC_DWC3_DEPCMD_STATUS_OK) {
		/*
		 * The controller refused it, so no transfer was started and no
		 * resource was taken.
		 */
		LOG_WRN("EP%02x Start Transfer had failed unobserved (0x%08x); "
			"returning the endpoint to idle", ep_data->cfg.addr, reg);
		udc_dwc3_ep_state_reset(ep_data);
		return;
	}

	udc_dwc3_adopt_xferrscidx(dev, ep_data, reg);
}

/*
 * Read the durable evidence for one endpoint and name the anomaly, if any.
 *
 * EVERY test here reads the TRB ring, an endpoint register or driver
 * bookkeeping. None reads an event, because the fault under investigation
 * destroys events - that constraint is the whole reason this function exists.
 */
static uint8_t udc_dwc3_recov_observe(const struct device *const dev,
				      struct udc_dwc3_ep_data *const ep_data)
{
	const uint32_t tail = ep_data->tail;
	const bool ring_live = (ep_data->head != tail) || ep_data->full;

	/*
	 * NOTHING IS OBSERVABLE ON AN ENDPOINT THAT IS NOT UP. This has to come
	 * before O5 and O4, because both read state that outlives a teardown.
	 * udc_dwc3_ep_disable() leaves the endpoint ENDING while it tears the
	 * endpoint down, so an endpoint being disabled looks exactly like one
	 * with a lost End Transfer completion - and the RESTART that follows
	 * would re-arm an endpoint the stack has just taken away.
	 */
	if (ep_data->trb_buf == NULL || !ep_data->cfg.stat.enabled) {
		return UDC_DWC3_RECOV_R_NONE;
	}

	/*
	 * Settle STARTING before anything below reads the state, so the
	 * heartbeat is the collector of last resort for a Start Transfer whose
	 * completion never arrived.
	 */
	udc_dwc3_ep_resolve_starting(dev, ep_data);

	/*
	 * O5 first: an endpoint stuck in ENDING cannot be armed, so every other
	 * observation below would be reading a ring that is frozen for a reason
	 * we already know.
	 */
	if (udc_dwc3_ep_is_ending(ep_data)) {
		return UDC_DWC3_RECOV_R_ENDXFER_LOST;
	}

	/*
	 * CONTROL ENDPOINTS ARE NOT RINGS, and must not be read as one. The
	 * control state machine addresses its descriptors as trb[0]/trb[1]
	 * directly: it never moves head/tail and never writes armed_len[]. So
	 * every ring test below is meaningless here - and worse than
	 * meaningless, because armed_len[] reads 0 for ever on EP0, which made
	 * the zero-length guard swallow the control path entirely and left O2
	 * and O3 unable to fire on the one endpoint whose wedge is terminal.
	 */
	/*
	 * The control observations below read EP0's descriptors directly, so
	 * they carry the same precondition as O2/O3:
	 */
	if (ep_data->xfer_state == UDC_DWC3_EP_STARTING ||
	    udc_dwc3_ep_is_ending(ep_data)) {
		return UDC_DWC3_RECOV_R_NONE;
	}

	if (USB_EP_GET_IDX(ep_data->cfg.addr) == 0U) {
		if (USB_EP_DIR_IS_OUT(ep_data->cfg.addr)) {
			/*
			 * O1: the controller is holding a SETUP the driver's
			 * armed descriptor is not taking.
			 */
			uint32_t sts = ep_data->trb_buf[0].status &
				       UDC_DWC3_TRB_STATUS_TRBSTS_MASK;

			if (sts == UDC_DWC3_TRB_STATUS_TRBSTS_OK &&
			    (ep_data->trb_buf[0].ctrl & UDC_DWC3_TRB_CTRL_CHN) != 0U) {
				sts = ep_data->trb_buf[1].status &
				      UDC_DWC3_TRB_STATUS_TRBSTS_MASK;
			}
			if (sts == UDC_DWC3_TRB_STATUS_TRBSTS_SETUPPENDING) {
				return UDC_DWC3_RECOV_R_NO_SETUP;
			}

			/*
			 * THE CONTROL MACHINE IS STOPPED WITH NOTHING LISTENING.
			 *
			 * The dwell in udc_dwc3_recov_dwell_ms() covers the brief gap
			 * between stages; a ring with no owned descriptor for
			 * UDC_DWC3_RECOV_SUSPECT_MS is stopped, not busy.
			 */
			for (uint32_t i = 0U; i < CONFIG_UDC_DWC3_TRB_NUM; i++) {
				if ((ep_data->trb_buf[i].ctrl &
				     UDC_DWC3_TRB_CTRL_HWO) != 0U) {
					return UDC_DWC3_RECOV_R_NONE;
				}
			}

			return UDC_DWC3_RECOV_R_NO_SETUP;
		}

		/*
		 * EP0-IN, O3: the controller still owns a descriptor that nothing is
		 * waiting on.
		 */
		if ((ep_data->trb_buf[0].ctrl & UDC_DWC3_TRB_CTRL_HWO) != 0U &&
		    !udc_ep_is_busy(&ep_data->cfg)) {
			return UDC_DWC3_RECOV_R_UNFETCHED;
		}

		return UDC_DWC3_RECOV_R_NONE;
	}

	/* O4: a live ring with no transfer resource. */
	/*
	 * O4: a live ring on an endpoint that is IDLE - nothing was ever started,
	 * or a Start Transfer was refused, so no arm can ever be processed.
	 */
	if (ring_live && ep_data->xfer_state == UDC_DWC3_EP_IDLE) {
		return UDC_DWC3_RECOV_R_NO_RESOURCE;
	}

	/*
	 * A descriptor armed with zero length is the CDC-ACM echo-mitigation
	 * ZLP.
	 */
	if (ep_data->armed_len[tail] == 0U) {
		return UDC_DWC3_RECOV_R_NONE;
	}

	/*
	 * IS ANYTHING ACTUALLY OUTSTANDING IN THIS SLOT? Hoisted, because O2 and O3
	 * below both need it and reading either without it is the same bug.
	 */
	/* ...AND ONLY WHILE THE CONTROLLER IS ACTUALLY RUNNING A TRANSFER. */
	/* udc_ep_is_busy() was a third term here and is removed. */
	if (ep_data->xfer_state != UDC_DWC3_EP_RUNNING ||
	    ep_data->net_buf[tail] == NULL) {
		return UDC_DWC3_RECOV_R_NONE;
	}

	/*
	 * O2: the controller finished with the descriptor and software never
	 * took the completion.
	 */
	if ((ep_data->trb_buf[tail].ctrl & UDC_DWC3_TRB_CTRL_HWO) == 0U) {
		return UDC_DWC3_RECOV_R_UNPOPPED;
	}

	/* O3: the controller owns the descriptor and is not fetching it. */
	if (!USB_EP_DIR_IS_IN(ep_data->cfg.addr)) {
		return UDC_DWC3_RECOV_R_NONE;
	}

	/*
	 * AN ENDPOINT THAT HAS NEVER RETIRED ANYTHING HAS NO BASELINE. Carried
	 * over from the old wedge detector, which excluded exactly this. "Armed,
	 * owned by the controller, nothing retired" is also what a first
	 * transfer looks like while it waits for a host that has not started
	 * polling this endpoint yet - before the application opens the device
	 * there may be no IN token for many seconds, and that is not a fault.
	 */
	if (ep_data->n_retire == 0U) {
		return UDC_DWC3_RECOV_R_NONE;
	}

	return UDC_DWC3_RECOV_R_UNFETCHED;
}

/*
 * Advance one endpoint's recovery by at most one action.
 *
 * Called once per heartbeat per endpoint, with the UDC mutex held. Returns true
 * when an action was taken, so the caller can keep to one controller action per
 * beat across the whole endpoint set.
 */
static bool udc_dwc3_recover_step(const struct device *const dev,
				  struct udc_dwc3_ep_data *const ep_data)
{
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	struct udc_dwc3_recov *const rc = &ep_data->recov;
	const bool is_ctrl = (USB_EP_GET_IDX(ep_data->cfg.addr) == 0U);
	const uint32_t now = k_cycle_get_32();
	uint8_t reason;

	/*
	 * FORWARD PROGRESS CLOSES THE EPISODE, and nothing else does. A retire
	 * is the only evidence the endpoint is moving again. Returning to IDLE
	 * on a timer, or because the symptom stopped being reported, is how a
	 * recovery budget gets silently refilled against an endpoint that never
	 * actually recovered.
	 */
	if (ep_data->n_retire != ep_data->recov_retire_mark) {
		ep_data->recov_retire_mark = ep_data->n_retire;

		if (rc->state != UDC_DWC3_RECOV_IDLE) {
			if (rc->state != UDC_DWC3_RECOV_FAILED) {
				rc->n_recovered++;
			} else if (rc->n_failed_retired < UINT8_MAX) {
				/*
				 * The budget was spent, but the endpoint has
				 * retired since - so it came back on its own.
				 * Recorded apart from n_failed, which stays as
				 * the count of episodes recovery could not
				 * finish.
				 */
				rc->n_failed_retired++;
			}
			LOG_INF("EP%02x: recovery %s -> idle after a retire (%u recovered, "
				"%u failed of which %u retired anyway, %u events discarded)",
				ep_data->cfg.addr,
				udc_dwc3_recov_state_name(rc->state),
				rc->n_recovered, rc->n_failed, rc->n_failed_retired,
				priv->recov_discard_seq);
			rc->state = UDC_DWC3_RECOV_IDLE;
			rc->reason = UDC_DWC3_RECOV_R_NONE;
			rc->attempts = 0U;
			rc->t0 = now;
		}
		return false;
	}

	/*
	 * An event the drain threw away may have been this endpoint's
	 * completion, its command completion or its control arm - the slot was
	 * empty, so which one is unknowable.
	 */
	if (ep_data->recov_discard_mark != priv->recov_discard_seq) {
		ep_data->recov_discard_mark = priv->recov_discard_seq;

		if (rc->state == UDC_DWC3_RECOV_IDLE) {
			rc->state = UDC_DWC3_RECOV_SUSPECT;
			rc->reason = UDC_DWC3_RECOV_R_SLOT_LOST;
			rc->t0 = now;
		}
	}

	/* FAILED is terminal until a retire. Say nothing more, touch nothing. */
	if (rc->state == UDC_DWC3_RECOV_FAILED) {
		return false;
	}

	/*
	 * AN EPISODE MUST NOT OUTLIVE THE ENDPOINT IT WAS OPENED ON.
	 *
	 * The action states below dispatch on rc->state alone; `reason` is read
	 * only by the IDLE and SUSPECT blocks. udc_dwc3_ep_disable() does not
	 * touch rc, so an episode opened while the endpoint was up keeps running
	 * after the stack has taken it away - and RESTART ends in
	 * udc_dwc3_ep_resume(), which with cfg.stat.enabled now false passes
	 * modify = false and so re-issues DEPXFERCFG on a recovery path, against
	 * INVARIANT 4, then re-sets DALEPENA and arms a Start Transfer on an
	 * endpoint the stack believes is disabled.
	 *
	 * A disabled endpoint has nothing to recover, so close the episode and
	 * give the next session a full budget.
	 */
	if (!ep_data->cfg.stat.enabled && rc->state != UDC_DWC3_RECOV_IDLE) {
		LOG_DBG("EP%02x: recovery %s -> idle, endpoint disabled",
			ep_data->cfg.addr, udc_dwc3_recov_state_name(rc->state));
		rc->state = UDC_DWC3_RECOV_IDLE;
		rc->reason = UDC_DWC3_RECOV_R_NONE;
		rc->attempts = 0U;
		rc->t0 = now;
		return false;
	}

	reason = udc_dwc3_recov_observe(dev, ep_data);

	/* ---------------- IDLE: watch only ---------------- */
	if (rc->state == UDC_DWC3_RECOV_IDLE) {
		if (reason == UDC_DWC3_RECOV_R_NONE) {
			return false;
		}
		rc->state = UDC_DWC3_RECOV_SUSPECT;
		rc->reason = reason;
		rc->t0 = now;
		return false;
	}

	/* ---------------- SUSPECT: let it persist, or drop it ---------------- */
	if (rc->state == UDC_DWC3_RECOV_SUSPECT) {
		if (reason == UDC_DWC3_RECOV_R_NONE) {
			rc->state = UDC_DWC3_RECOV_IDLE;
			rc->reason = UDC_DWC3_RECOV_R_NONE;
			return false;
		}
		if (reason != rc->reason) {
			/* The evidence changed - re-time against the new one. */
			rc->reason = reason;
			rc->t0 = now;
			return false;
		}

		/*
		 * Sampled here, not at function entry. IDLE does not refresh t0, and
		 * the discard pickup above can move the machine into SUSPECT on this
		 * very beat - an age taken at entry would then measure however long
		 * the endpoint had been sitting in IDLE and clear the dwell gate
		 * immediately, acting on evidence one beat old.
		 */
		if (k_cyc_to_ms_near32(now - rc->t0) <
		    udc_dwc3_recov_dwell_ms(ep_data, rc->reason)) {
			return false;
		}

		if (rc->attempts >= UDC_DWC3_RECOV_MAX_ACTIONS) {
			udc_dwc3_recov_fail(dev, ep_data,
					    "action budget spent, endpoint left as it is "
					    "so the state stays readable");
			return false;
		}

		/* Choose the state that matches the evidence. One mapping, here. */
		switch (rc->reason) {
		case UDC_DWC3_RECOV_R_UNPOPPED:
			rc->state = UDC_DWC3_RECOV_RESYNC;
			break;
		case UDC_DWC3_RECOV_R_UNFETCHED:
			/*
			 * RECLAIM for every endpoint that can reach here, EP0-IN
			 * included - taking the descriptor back with End
			 * Transfer is exactly what the old EP0-IN reclaim did,
			 * and it is safe on an IN endpoint.
			 */
			rc->state = UDC_DWC3_RECOV_RECLAIM;
			break;
		case UDC_DWC3_RECOV_R_NO_RESOURCE:
			rc->state = UDC_DWC3_RECOV_RESTART;
			break;
		case UDC_DWC3_RECOV_R_ENDXFER_LOST:
			rc->state = UDC_DWC3_RECOV_RESTART;
			break;
		case UDC_DWC3_RECOV_R_NO_SETUP:
			rc->state = UDC_DWC3_RECOV_STALL;
			break;
		default:
			/*
			 * Unreachable: every reason the observer can return has
			 * a case above, and SLOT_LOST is only ever a seed that
			 * the next beat replaces.
			 */
			rc->state = UDC_DWC3_RECOV_IDLE;
			rc->reason = UDC_DWC3_RECOV_R_NONE;
			return false;
		}
		rc->t0 = now;

		/*
		 * Capped the same way the give-up report is. An episode ends at
		 * the next retire, so a flapping endpoint opens them
		 * continuously:
		 */
		if (rc->n_recovered + rc->n_failed <= UDC_DWC3_RECOV_REPORT_MAX) {
			LOG_ERR("EP%02x: recovery %s - %s", ep_data->cfg.addr,
				udc_dwc3_recov_state_name(rc->state),
				udc_dwc3_recov_reason_name(rc->reason));
		}
		/* Fall through into the action below on THIS call. */
	}

	/* ---------------- RESYNC: take the completion software missed -------- */
	if (rc->state == UDC_DWC3_RECOV_RESYNC) {
		const uint32_t got = udc_dwc3_drain_completed(dev, ep_data);

		rc->attempts++;
		rc->state = UDC_DWC3_RECOV_SUSPECT;
		rc->t0 = now;

		if (got > 0U) {
			priv->evt_sweep_rescued += got;
			priv->evt_sweep_runs++;
			return true;
		}

		/*
		 * O2 and udc_dwc3_pop_trb() test the same two conditions - a
		 * parked net_buf and HWO clear - and both run under this mutex,
		 * so a resync that takes nothing means the observation was
		 * wrong, not that the endpoint recovered.
		 */
		LOG_ERR("EP%02x: resync took nothing - O2 fired with no completion to "
			"take; the observation is wrong", ep_data->cfg.addr);
		return true;
	}

	/* ---------------- RECLAIM: make the controller let go ---------------- */
	if (rc->state == UDC_DWC3_RECOV_RECLAIM) {
		/*
		 * End Transfer is the only legal way to take back a descriptor
		 * the controller owns:
		 */
		if (!udc_dwc3_depcmd_end_xfer(dev, ep_data,
					      UDC_DWC3_DEPCMD_HIPRI_FORCERM)) {
			udc_dwc3_recov_fail(dev, ep_data,
					    "End Transfer would not issue, endpoint stays held");
			return false;
		}
		rc->attempts++;
		/*
		 * Wait for the completion in RESTART, where CMDCMPLT_MS bounds it. A
		 * lost EpCmdCmplt therefore costs one timeout, not the endpoint.
		 */
		rc->state = UDC_DWC3_RECOV_RESTART;
		rc->t0 = now;
		return true;
	}

	/* ---------------- RESTART: re-establish the transfer ----------------- */
	if (rc->state == UDC_DWC3_RECOV_RESTART) {
		/*
		 * Time spent in RESTART, recomputed. `age` was sampled against the
		 * PREVIOUS state's t0 at entry, and the SUSPECT block above falls
		 * through into here after setting a fresh t0 - so using it would time
		 * the wait for a Command Complete against however long SUSPECT sat,
		 * and could declare the completion lost on the very first beat here
		 * without ever having waited for it.
		 */
		const uint32_t wait_ms = k_cyc_to_ms_near32(now - rc->t0);

		if (udc_dwc3_ep_is_ending(ep_data)) {
			if (wait_ms < UDC_DWC3_RECOV_CMDCMPLT_MS) {
				return false;	/* still legitimately in flight */
			}
			/*
			 * THE COMPLETION NEVER CAME. Clearing the flag here is
			 * what stops one lost event from ending the endpoint;
			 */
			udc_dwc3_ep_state_reset(ep_data);
			LOG_ERR("EP%02x: End Transfer completion lost (%u ms) - releasing "
				"it rather than leaving the endpoint unrecoverable",
				ep_data->cfg.addr, wait_ms);
		}

		if (is_ctrl) {
			/* EP0 stages are armed by the control machine, not a ring. */
			memset((void *)ep_data->trb_buf, 0,
			       sizeof(*ep_data->trb_buf) *
					(CONFIG_UDC_DWC3_TRB_NUM - 1));
			udc_ep_set_busy(&ep_data->cfg, false);
			udc_dwc3_ctrl_next(dev);
		} else {
			uint32_t cmdreg = 0U;

			/*
			 * PROOF BEFORE RELEASE. THE ABSENCE OF AN EVENT IS NOT THE
			 * PRESENCE OF QUIESCENCE.
			 */
			if (!udc_dwc3_wait_cmdact_zero(dev,
						       UDC_DWC3_DEPCMD(ep_data->epn),
						       &cmdreg)) {
				udc_dwc3_recov_fail(dev, ep_data,
						    "End Transfer still executing (CMDACT set): "
						    "ring NOT released, buffers left with the "
						    "controller");
				return true;
			}

			/*
			 * ONE Start Transfer, not two. udc_dwc3_ep_resume()
			 * reaches udc_dwc3_trb_nonctrl_init(), which issues the
			 * Start Transfer itself - "Start the transfer now,
			 * update it later".
			 */
			udc_dwc3_ep_ring_release(ep_data);

			if (udc_dwc3_ep_resume(dev, ep_data,
					       ep_data->cfg.stat.enabled) != 0) {
				udc_dwc3_recov_fail(dev, ep_data,
						    "resume did not re-establish the "
						    "transfer");
				return true;
			}
		}
		rc->attempts++;
		rc->state = UDC_DWC3_RECOV_SUSPECT;
		rc->t0 = now;
		return true;
	}

	/* ---------------- STALL: control path only --------------------------- */
	if (rc->state == UDC_DWC3_RECOV_STALL) {
		/*
		 * LIGHTEST THING THAT CAN WORK FIRST. The common cause of
		 * "nothing owned on EP0-OUT" is a stale claim: the SETUP was
		 * consumed and its lost XferComplete never released
		 * udc_ep_is_busy(), so udc_dwc3_ctrl_try() declines every arm
		 * from then on.
		 */
		if (rc->attempts == 0U) {
			udc_dwc3_ctrl_reset_to_step1(dev, false);
			udc_dwc3_ctrl_next(dev);
		} else {
			(void)udc_dwc3_recover(dev);
		}

		rc->attempts++;
		rc->state = UDC_DWC3_RECOV_SUSPECT;
		rc->t0 = now;
		return true;
	}

	return false;
}

/*
 * Record an event the drain had to throw away, and make the endpoints resync.
 *
 * The old dead-slot skip credited the slot inside the drain and the loss then
 * vanished: nothing counted it, nothing knew WHICH endpoint had just lost its
 * completion, and the endpoint's later death looked like a fresh fault. A
 * discarded event may have been an XferComplete (the buffer is never retired),
 * an EpCmdCmplt (the resource index is never adopted) or an EP0 XferNotReady
 * (the stage is never armed) - and since the slot is empty, we cannot know
 * which. So every endpoint is pushed back to SUSPECT and re-observed from
 * durable state, which is the only honest response to "something was lost".
 */
static void udc_dwc3_recov_note_discard(const struct device *const dev)
{
	struct udc_dwc3_data *const priv = udc_get_private(dev);

	/*
	 * A COUNTER, NOT A STATE WRITE. This runs on the event-drain thread and
	 * holds no lock, while udc_dwc3_recover_step() reads and writes recov.*
	 * on the heartbeat thread under the UDC mutex.
	 */
	priv->recov_discard_seq++;
}

/*
 * The one call site. Walks every endpoint and allows ONE controller action per
 * beat across the whole set, so a bad beat cannot turn into a command storm.
 */
static void udc_dwc3_recover_all(const struct device *const dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;

	for (int i = 0; i < cfg->num_in_eps; i++) {
		if (udc_dwc3_recover_step(dev, &cfg->ep_data_in[i])) {
			return;
		}
	}
	for (int i = 0; i < cfg->num_out_eps; i++) {
		if (udc_dwc3_recover_step(dev, &cfg->ep_data_out[i])) {
			return;
		}
	}
}

/*
 * Periodic liveness work: observe the drain, run the recovery machine, report.
 */
static void udc_dwc3_heartbeat_worker(struct k_work *work)
{

	struct udc_dwc3_data *const priv =
		CONTAINER_OF(work, struct udc_dwc3_data, heartbeat_work);
	const struct device *const dev = priv->dev;
	const struct udc_dwc3_config *const cfg = dev->config;
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	const uint32_t d_evt = priv->dispatch_evt;
	/*
	 * NOT A READ: udc_dwc3_drain_helper() performed the heartbeat's single
	 * read this beat. Everything below uses the published value.
	 */
	const uint32_t gc = priv->evt_gc_last;
	const uint32_t hb_now = k_cycle_get_32();

	/* Every run counts itself before anything here can block or return. */
	if (priv->hb_beats != 0U) {
		const uint32_t gap = k_cyc_to_ms_near32(hb_now - priv->hb_last_t);

		if (gap > priv->hb_gap_ms_max) {
			priv->hb_gap_ms_max = gap;
		}
	}
	priv->hb_last_t = hb_now;
	priv->hb_beats++;

	/*
	 * REPORTED ON A TIMER, NOT AN EVENT COUNT.
	 *
	 * This used to fire on evt_handled % UDC_DWC3_EVT_STATS_EVERY, from the
	 * drain thread. evt_handled is exactly what STOPS advancing during an
	 * event-loss episode, so the one line that reports the loss fell silent
	 * precisely while the fault was happening and the last line emitted was
	 * a pre-fault snapshot reading late 0 gaveup 0.
	 *
	 * It also belongs off the drain thread: roughly 700 characters at
	 * ~87 us/char under CONFIG_LOG_MODE_MINIMAL is about 60 ms of
	 * synchronous uart_poll_out during which the ring is not drained.
	 */
	if ((priv->hb_beats % UDC_DWC3_EVT_STATS_BEATS) == 0U) {
		/*
		 * ctrl_desync is here because its log line is capped at the
		 * first UDC_DWC3_CTRL_DESYNC_LOG_FIRST occurrences - without a
		 * counter in this line, a check firing thousands of times looks
		 * exactly like one firing twelve times.
		 */
		/*
		 * Stack headroom, measured not assumed. The drain thread
		 * gets UDC_DWC3_EVT_STACK_SIZE bytes and a synchronous log
		 * backend formats on it, so this is the number that says
		 * whether 512 was the right call.
		 */
		if (IS_ENABLED(CONFIG_INIT_STACKS) &&
		    IS_ENABLED(CONFIG_THREAD_STACK_INFO) &&
		    priv->evt_thread != NULL) {
			size_t used = 0;

			if (k_thread_stack_space_get(priv->evt_thread,
						     &used) == 0) {
				const uint32_t free =
					UDC_DWC3_EVT_STACK_SIZE - (uint32_t)used;

				if (free < priv->evt_stack_free) {
					priv->evt_stack_free = free;
				}
			}
		}

		LOG_INF("evtstack %u B free of %u", priv->evt_stack_free,
			UDC_DWC3_EVT_STACK_SIZE);

		LOG_INF("events %u: late %u (worst %u polls/%u us) gaveup %u "
			"(worst empty %u us) isr %u runs %u rearm %u kick %u skip %u "
			"zero %u missed %u/%u desync %u ctrl %u/%u midzero %u "
			"decline %u/%u "
			"setuppending %u startfail %u defer %u trbsts %u "
			"setupwd %u/%u reset %u updxfer %u recache %u outmisaligned %u/%u xnrdy %u "
			"statuswait %u stall %u ephalt %u "
			"gc_hwm %u B gc0max %u B multi %u link %u out1 %u/%u out2 %u/%u stomp %u DSTS 0x%08x",
			priv->evt_handled, priv->evt_late,
			priv->evt_late_polls_max, priv->evt_late_us_max,
			priv->evt_gaveup, priv->evt_gaveup_us_max,
			priv->evt_isr, priv->evt_worker_runs, priv->evt_rearm,
			priv->evt_kick, priv->evt_skipped,
			priv->evt_zero, priv->evt_missed, priv->evt_missed_frozen,
			priv->ctrl_desync,
			priv->ctrl_setup_done, priv->ctrl_status_done,
			priv->evt_midzero, priv->ctrl_decline, priv->ctrl_recover,
			priv->ctrl_setup_pending, priv->ctrl_start_fail,
			priv->ctrl_deferred_arm, priv->ctrl_trbsts_other,
			priv->ctrl_setup_wd_fire, priv->ctrl_setup_wd_idle,
			priv->ctrl_setup_wd_reset,
			priv->ctrl_setup_wd_updxfer,
			priv->nonctrl_recache,
			priv->out_unaligned, priv->out_unaligned_ctrl,
			priv->xnrdy_nonctrl,
			priv->ctrl_status_defer, priv->ctrl_stall_issued, priv->ep_halts,
			priv->evt_gevntcount_hwm,
			priv->evt_gaveup_gc0_max, priv->evt_gaveup_multi,
			priv->evt_link_total,
			cfg->num_out_eps > 1 ? cfg->ep_data_out[1].n_arm : 0U,
			cfg->num_out_eps > 1 ? cfg->ep_data_out[1].n_retire : 0U,
			cfg->num_out_eps > 2 ? cfg->ep_data_out[2].n_arm : 0U,
			cfg->num_out_eps > 2 ? cfg->ep_data_out[2].n_retire : 0U,
			priv->trb_stomp,
			/*
			 * DSTS on the periodic line, so COREIDLE and
			 * RXFIFOEMPTY get sampled during HEALTHY
			 * STREAMING - the one case never captured.
			 */
			sys_read32(base + UDC_DWC3_DSTS));
	}

	if (priv->hb_submit_t != 0U) {
		const uint32_t q = k_cyc_to_ms_near32(hb_now - priv->hb_submit_t);

		if (q > priv->hb_q_ms_max) {
			priv->hb_q_ms_max = q;
		}

		priv->hb_submit_t = 0U;
	}

	/*
	 * THE ONE RECOVERY CALL SITE. Every recovery this driver performs is
	 * reached from here, through the state machine in udc_dwc3_recover_step().
	 * Nothing else in the file restarts, reclaims or discards a transfer.
	 */
	{

		udc_lock_internal(dev, K_FOREVER);
		udc_dwc3_recover_all(dev);
		udc_unlock_internal(dev);
	}
	/*
	 * How long the drain has been parked on the same empty slot, or 0 when
	 * it is not parked at all.
	 */
	const uint32_t gaveup_ms = priv->drain.attempts > 0U
		? k_cyc_to_ms_near32(k_cycle_get_32() - priv->drain.since)
		: 0U;
	const uint32_t idle_ms =
		k_cyc_to_ms_near32(k_cycle_get_32() - priv->evt_worker_exit_t0);
	bool drain_stuck;

	if (gc > 0U && priv->evt_handled == priv->hb_last_handled) {
		priv->hb_drain_stuck_beats++;
	} else {
		priv->hb_drain_stuck_beats = 0U;
	}
	priv->hb_last_handled = priv->evt_handled;

	drain_stuck = (priv->hb_drain_stuck_beats * UDC_DWC3_HEARTBEAT_MS) >=
		      UDC_DWC3_HB_DRAIN_STUCK_MS;

	/*
	 * Report the bus/DMA configuration once, from here rather than from
	 * udc_dwc3_init().
	 */
	if (!priv->buscfg_logged) {
		priv->buscfg_logged = true;
		LOG_INF("BUSCFG: GSBUSCFG0=0x%08x GSBUSCFG1=0x%08x GUCTL=0x%08x "
			"GUCTL1=0x%08x GCTL=0x%08x",
			sys_read32(base + UDC_DWC3_GSBUSCFG0),
			sys_read32(base + UDC_DWC3_GSBUSCFG1),
			sys_read32(base + UDC_DWC3_GUCTL),
			sys_read32(base + UDC_DWC3_GUCTL1),
			sys_read32(base + UDC_DWC3_GCTL));
	}

	/* CORE debug baseline. */
	if (++priv->core_dbg_beats >= UDC_DWC3_CORE_DBG_BEATS) {
		struct udc_dwc3_core_dbg dbg;

		priv->core_dbg_beats = 0U;
		udc_dwc3_core_dbg_read(base, &dbg);
		udc_dwc3_core_dbg_log(" hb", &dbg);

		/*
		 * Beats since boot and the worst gap so far. Successive lines
		 * must differ by exactly UDC_DWC3_CORE_DBG_BEATS;
		 */
		LOG_INF("  HB: beats %u/%u drop %u gapmax %u ms qmax %u ms "
			"(nominal %u)",
			priv->hb_beats, priv->hb_expiries, priv->hb_coalesced,
			priv->hb_gap_ms_max, priv->hb_q_ms_max,
			UDC_DWC3_HEARTBEAT_MS);
	}

	/* Kick a handler that has stopped being scheduled. */
	/*
	 * The kick itself has moved into udc_dwc3_heartbeat_expiry(), which sees
	 * the same condition one queue-hop earlier.
	 */

	/* Break a control claim that will never be given back. */
	/*
	 * A SETUP ARMED ON EP0-OUT IS NEVER A WEDGE, however old the claim and
	 * however many declines have piled up behind it.
	 */
	/*
	 * ...OR the drain has stopped, which is the case the clause above misses.
	 */
	/*
	 * STANDS DOWN WHILE THE STATE MACHINE OWNS EP0. This and
	 * udc_dwc3_recover_step() watch the same symptom - the host is asking
	 * and no SETUP is armed - and reach for the same action,
	 * udc_dwc3_recover(). udc_dwc3_recover_all() has already run earlier in
	 * this very function, so without this test both could fire in one beat
	 * and issue two Set Stall / re-arm sequences back to back on the control
	 * endpoint.
	 */
	/*
	 * A STUCK DRAIN SUPPRESSES THIS DETECTOR; IT DOES NOT OVERRIDE IT.
	 *
	 * drain_stuck used to be an OR against both guards below, which disabled
	 * the two conditions that exist to say "EP0 is fine": the recovery
	 * machine already owns it, and a SETUP is armed and waiting for the
	 * host. When the event ring stalls, the control claim is held BECAUSE
	 * events are not being delivered - it is a symptom, not an independent
	 * fault. Recovering EP0 cannot deliver a missing event; it issues Set
	 * Stall and a fresh SETUP, so the host re-drives the transfer and the
	 * controller emits MORE events into a ring that is already not draining.
	 * Measured: 0 firings on a healthy ring, 27 on a stalled one, 10 of them
	 * citing the stuck drain as their reason.
	 */
	if (priv->ctrl_decline_pending && !drain_stuck &&
	    !udc_dwc3_recov_owns(&cfg->ep_data_out[0]) &&
	    !udc_dwc3_recov_owns(&cfg->ep_data_in[0]) &&
	    !udc_dwc3_ctrl_armed_setup(&cfg->ep_data_out[0]) &&
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
		 * udc_dwc3_recover() issues Set Stall on EP0-OUT. It does NOT
		 * issue an End Transfer - an earlier version of this comment
		 * claimed it did, and that was wrong.
		 */
		/*
		 * recover() owns the stall. It issues Set Stall on EP0-OUT in
		 * the common case, and deliberately does NOT when it finds no
		 * control stage outstanding - which is exactly the state a soft
		 * reset leaves behind.
		 */
		/*
		 * ACT ON THE FAULT DIAGNOSED, not on the endpoint that reported it.
		 */
		if (drain_stuck) {
			priv->evt_kick++;
			k_sem_give(&priv->evt_sem);
		} else {
			(void)udc_dwc3_recover(dev);
		}
	}

	/*
	 * Report the moment control traffic stops, with the state that decides
	 * WHY.
	 */
	if (priv->ctrl_setup_done != priv->hb_last_setup_done) {
		priv->hb_last_setup_done = priv->ctrl_setup_done;
		/* Control traffic moved: if a stall was captured, take its pair. */
		udc_lock_internal(dev, K_FOREVER);
		udc_dwc3_ctrl_stall_cleared(dev);
		udc_unlock_internal(dev);
		priv->ctrl_quiet_t0 = k_cycle_get_32();
		priv->ctrl_quiet_logged = false;
	} else if (!priv->ctrl_quiet_logged && priv->ctrl_setup_done > 0U &&
		   priv->ctrl_decline_pending &&
		   /*
		    * The decline must fall INSIDE the silence, not merely before it.
		    * ctrl_decline_pending is a level - set on a decline, cleared on a
		    * grant - so when enumeration ends on a declined request it stays set
		    * across an idle bus, and this fired once on every healthy boot at a
		    * cost of about 32 lines of stall dump.
		    */
		   (int32_t)(priv->ctrl_decline_t - priv->ctrl_quiet_t0) >= 0 &&
		   k_cyc_to_ms_near32(k_cycle_get_32() - priv->ctrl_quiet_t0) >=
						UDC_DWC3_CTRL_QUIET_MS) {
		/* ctrl_decline_pending is the whole point of this clause: */
		priv->ctrl_quiet_logged = true;
		/*
		 * INFO, not ERROR. This fires on control-endpoint IDLENESS, and
		 * the device cannot tell an idle host from a device that has
		 * stopped receiving - rxfifoempty=1 and gc=0 look identical in
		 * both cases.
		 */
		LOG_INF("no control traffic for %u ms after %u SETUPs: busy o/i %u/%u, "
			"trb o/i 0x%08x/0x%08x, gc %u B, DSTS 0x%08x (rxfifoempty %u), "
			"decline %u, "
			"setuppend %u, ep0out queued %u, state %u<-%u seq %u",
			UDC_DWC3_CTRL_QUIET_MS, priv->ctrl_setup_done,
			udc_ep_is_busy(&cfg->ep_data_out[0].cfg) ? 1U : 0U,
			udc_ep_is_busy(&cfg->ep_data_in[0].cfg) ? 1U : 0U,
			cfg->ep_data_out[0].trb_buf[0].ctrl,
			cfg->ep_data_in[0].trb_buf[0].ctrl,
			gc, sys_read32(base + UDC_DWC3_DSTS),
			(sys_read32(base + UDC_DWC3_DSTS) &
			 UDC_DWC3_DSTS_RXFIFOEMPTY) ? 1U : 0U,
			priv->ctrl_decline, priv->ctrl_setup_pending,
			udc_buf_peek(&cfg->ep_data_out[0].cfg) != NULL ? 1U : 0U,
			(unsigned int)priv->ctrl_state, priv->ctrl_state_prev,
			priv->ctrl_state_seq);

#ifdef STALL_DIAG_LOG
		/*
		 * LOCKED, for the same reason as the
		 * udc_dwc3_ctrl_stall_cleared() call above:
		 */
		udc_lock_internal(dev, K_FOREVER);
		udc_dwc3_stall_diag_dump(dev, "control traffic stopped");
		udc_unlock_internal(dev);
#endif

		/*
		 * Non-control endpoints get a re-cache here, and ONLY here. HWO
		 * stays 1 and BUFSIZ keeps the software-prepared value in both
		 * cases, because the controller only writes the descriptor back
		 * when it retires it - databook 4.2.3, "when the hardware writes
		 * back the TRBs, it updates the BUFSIZ field to represent the
		 * remaining unused buffer".
		 */
		/* F4: under the mutex. */
		udc_lock_internal(dev, K_FOREVER);

		for (int i = 1; i < cfg->num_out_eps; i++) {
			struct udc_dwc3_ep_data *const e = &cfg->ep_data_out[i];
			const volatile struct udc_dwc3_trb *t = e->trb_buf;

			if (t == NULL || !udc_ep_is_busy(&e->cfg) ||
			    (t[e->tail].ctrl & UDC_DWC3_TRB_CTRL_HWO) == 0U) {
				continue;
			}

			priv->nonctrl_recache++;
			udc_dwc3_depcmd_update_xfer(dev, e);

			LOG_WRN("  EP%02x armed through a control stall (ctrl 0x%08x "
				"sts 0x%08x): re-cached (%u)", e->cfg.addr,
				t[e->tail].ctrl, t[e->tail].status,
				priv->nonctrl_recache);
		}

		udc_unlock_internal(dev);
	}

	/*
	 * THE EP0-OUT ALWAYS-ARMED INVARIANT CHECK WAS REMOVED HERE, and must
	 * not come back in this form.
	 */

	if (d_evt != 0U) {
		const uint32_t ms =
			k_cyc_to_ms_near32(k_cycle_get_32() - priv->dispatch_t0);

		if (ms >= UDC_DWC3_DISPATCH_STUCK_MS) {
			LOG_ERR_RATELIMIT("dispatch stuck %u ms in %s (evt 0x%08x)", ms,
				udc_dwc3_get_event_name(d_evt,
					sys_read32(base + UDC_DWC3_DSTS)), d_evt);
		}
	} else if (gc > 0U && priv->drain.state != UDC_DWC3_DRAIN_WAITING &&
		   (gaveup_ms >= UDC_DWC3_EVT_GAVEUP_AGE_MS ||
				priv->evt_handled == priv->hb_last_evt_handled)) {
		/*
		 * NOT WHILE THE DRAIN IS INSIDE A WAIT. Both reports below describe a
		 * drain that has stopped - "IDLE" says the worker never ran, "NOT
		 * ADVANCING" says it is parked - and neither is true of a thread that
		 * is deliberately asleep between two looks at the slot.
		 */
		/*
		 * Two different ways the ring stops being drained, and it takes both
		 * tests to see them.
		 */
		/*
		 * The slot CONTENTS are what separate the two ways this happens,
		 * and the slot number alone cannot:
		 */
		/*
		 * How LONG the slot has been empty is what settles it, not how
		 * many times we looked.
		 */
		/*
		 * Name the condition that fired. Two can, and they mean opposite
		 * things:
		 */
		if (priv->drain.attempts > 0U) {
			LOG_ERR_RATELIMIT("%u B pending, drain NOT ADVANCING on slot %u for "
				"%u ms over %u give-ups (handled %s since last "
				"beat): slot holds 0x%08x, DSTS=0x%08x",
				gc, priv->evt_next, gaveup_ms, priv->drain.attempts,
				priv->evt_handled == priv->hb_last_evt_handled ?
					"nothing" : "events",
				cfg->evt_buf[priv->evt_next],
				sys_read32(base + UDC_DWC3_DSTS));

			/*
			 * REPORTING ONLY from here. The decision and the action
			 * moved to udc_dwc3_drain_slot_is_dead(), called by the
			 * drain, because acting meant writing priv->evt_next
			 * from this thread while the drain thread was writing it
			 * too.
			 */
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
		k_sem_give(&priv->evt_sem);
	}

	priv->hb_last_evt_handled = priv->evt_handled;

	/* No re-arm here on purpose: the periodic timer owns the cadence. */
}

#ifdef UDC_DWC3_SETUP_STUCK_RESET
/*
 * Last resort for a SETUP the controller has received and will not retire.
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

	/*
	 * The ring position and the drain state are reset together - they
	 * describe the same thing and must not survive one another.
	 */
	priv->evt_next = 0;
	udc_dwc3_drain_reset(&priv->drain);

	/*
	 * Withdraw the nudge with the state that asked for it. A request
	 * submitted before this point would otherwise run after the core soft
	 * reset below and write DGCMD on a controller that is mid-reset.
	 */
	(void)k_work_cancel(&priv->nudge_work);

	udc_dwc3_disable(dev);

	/*
	 * shutdown() is not optional here. udc_dwc3_disable() stops the timer,
	 * clears RunStop and masks the IRQ, but it does NOT disable the
	 * endpoints - udc_ep_config.stat.enabled stays set.
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
	const struct udc_dwc3_config *const wd_cfg = dev->config;
	struct udc_dwc3_ep_data *wd_ep;

	/*
	 * A SETUP TRB is armed speculatively and then waits on the host, so its
	 * age says nothing - the bus can sit idle for minutes with the endpoint
	 * perfectly healthy.
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
		 * The SETUP TRB is the only endpoint-specific evidence
		 * available.
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
		 * Occupancy alone is not evidence. There is one RxFIFO for every
		 * OUT endpoint in device mode, so with a bulk OUT endpoint
		 * enabled - CDC+Video enables several - RXFIFOEMPTY can be clear
		 * because of someone else's data while EP0-OUT waits, perfectly
		 * healthy, for a SETUP that has not arrived.
		 */
		if (moved) {
			priv->ctrl_setup_wd_busy++;
			priv->ctrl_setup_wd_snap_setup = priv->ctrl_setup_done;
			priv->ctrl_setup_wd_snap_nonctrl = priv->nonctrl_done;
			k_work_reschedule(&priv->watchdog_dwork,
					  K_MSEC(CONFIG_UDC_DWC3_RECOVERY_TIMEOUT));
			return;
		}

		/*
		 * The state machine may already be working this endpoint - its O1
		 * watches TRBSTS for a SETUP the controller cannot deliver, which is
		 * the same wedge seen from the other side, and its STALL state ends in
		 * the same Set Stall on EP0-OUT that this is about to issue.
		 */
		{
			/*
			 * Read under the mutex. This worker is on the SYSTEM
			 * workqueue while udc_dwc3_recover_step() runs on the UDC
			 * work queue, so an unlocked read could see the machine idle
			 * one instruction before it opens an episode - which is the
			 * one case this guard exists to prevent. Taken and released
			 * around the read alone, so no path below returns holding it.
			 */
			bool machine_owns;

			udc_lock_internal(dev, K_FOREVER);
			machine_owns = udc_dwc3_recov_owns(ep0_out);
			udc_unlock_internal(dev);

			if (machine_owns) {
				priv->ctrl_setup_wd_busy++;
				k_work_reschedule(&priv->watchdog_dwork,
						  K_MSEC(CONFIG_UDC_DWC3_RECOVERY_TIMEOUT));
				return;
			}
		}

		priv->ctrl_setup_wd_fire++;
		LOG_ERR("SETUP outstanding for %u ms, TRB still owned by the core and "
			"nothing retired meanwhile (DSTS 0x%08x, TRB ctrl 0x%08x): a "
			"received SETUP has not been retired, stalling EP0-OUT (fired "
			"%u; suppressed idle %u, busy %u, retired %u)",
			CONFIG_UDC_DWC3_RECOVERY_TIMEOUT, dsts, trb_ctrl,
			priv->ctrl_setup_wd_fire, priv->ctrl_setup_wd_idle,
			priv->ctrl_setup_wd_busy, priv->ctrl_setup_wd_retired);

		/* Read the core's own state before anything is done to it. */
		udc_dwc3_core_state_dump(dev);

		/*
		 * Last, because it is the only part of this that issues a
		 * command.
		 */
		{
			const uint32_t epn = ep0_out->epn;

			/*
			 * Under the lock. This worker runs on the SYSTEM
			 * workqueue, not udc_get_work_q(), so it is concurrent
			 * with the usbd thread, which reaches
			 * udc_dwc3_depcmd_start_xfer() through ep_enqueue on
			 * these same endpoint command registers.
			 */
			udc_lock_internal(dev, K_FOREVER);
			udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(epn),
					UDC_DWC3_DEPCMD_DEPGETSTATE);
			udc_unlock_internal(dev);

			LOG_INF("  CORE: EP0-OUT EPSTATE=0x%08x",
				sys_read32(base + UDC_DWC3_DEPCMDPAR2(epn)));
		}

		/*
		 * Cheapest recovery first, and the one that tells us what this
		 * is. Databook 3.2.2.6:
		 */
		if (priv->ctrl_setup_done != priv->ctrl_setup_wd_upd_mark ||
		    priv->ctrl_setup_wd_updxfer == 0U) {
			priv->ctrl_setup_wd_updxfer++;
			priv->ctrl_setup_wd_upd_mark = priv->ctrl_setup_done;

			/* Locked for the reason given at the DEPGETSTATE above. */
			bool issued;

			udc_lock_internal(dev, K_FOREVER);
			issued = udc_dwc3_depcmd_update_xfer(dev, ep0_out);
			udc_unlock_internal(dev);

			if (!issued) {
				/*
				 * Refused for want of a transfer resource index.
				 * Say so rather than claim a re-cache, and give
				 * the count back:
				 */
				priv->ctrl_setup_wd_updxfer--;
				LOG_WRN("  Update Transfer on EP0-OUT refused: no "
					"transfer resource index established");
			} else {
			LOG_WRN("  re-cached the EP0-OUT descriptor with Update Transfer "
				"(attempt %u): if the SETUP retires now, the core was "
				"holding a stale HWO=0 and this is a descriptor "
				"visibility fault, not a controller refusal",
				priv->ctrl_setup_wd_updxfer);
			}

			k_work_reschedule(&priv->watchdog_dwork,
					  K_MSEC(CONFIG_UDC_DWC3_RECOVERY_TIMEOUT));
			return;
		}

#ifdef UDC_DWC3_SETUP_STUCK_RESET
		/*
		 * Two fires with no SETUP retired between them means the Set
		 * Stall issued last time did not clear it.
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
	 * A DATA or STATUS stage times out and comes straight here, and until
	 * now that path recorded NOTHING - so the one moment control actually
	 * broke was the one moment with no state captured, and the last three
	 * wedges had to be reconstructed backwards from the tail of the log.
	 */
	/*
	 * Snapshot watchdog_ep ONCE. This worker runs on the system work queue
	 * and reads it without the UDC mutex, while udc_dwc3_on_ctrl(),
	 * ep_disable() and drop_xfer_state() all clear it from the UDC work
	 * queue. The endpoint objects are static (cfg->ep_data_*), so a snapshot
	 * cannot dangle - at worst it names an endpoint whose stage has just
	 * completed, and Update Transfer against a completed resource is
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
			"setup %u status %u decline %u | state %u<-%u seq %u | DSTS 0x%08x",
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
			(unsigned int)priv->ctrl_state, priv->ctrl_state_prev,
			priv->ctrl_state_seq,
			sys_read32(base + UDC_DWC3_DSTS));
	}

	/*
	 * Try the cheap remedy on THIS stage before Set Stall, whatever stage it
	 * is. databook 3.2.2.6 says issuing it against a resource that has
	 * already completed is detected and ignored, so it cannot do the damage
	 * that ending a control transfer did.
	 */
	/*
	 * Only for a stage that was actually armed. udc_dwc3_ctrl_try() also
	 * schedules this watchdog for DEFERRALS - a status wait, or a wait for
	 * an End Transfer - with watchdog_type NONE and nothing armed on the
	 * endpoint.
	 */
	if (wd_ep != NULL && priv->watchdog_type != UDC_DWC3_WATCHDOG_TYPE_NONE &&
	    (priv->ctrl_setup_done + priv->ctrl_status_done) != priv->ctrl_wd_upd_mark) {
		priv->ctrl_wd_upd_mark = priv->ctrl_setup_done + priv->ctrl_status_done;
		priv->ctrl_setup_wd_updxfer++;

		/* Locked for the reason given at the DEPGETSTATE above. */
		bool issued;

		udc_lock_internal(dev, K_FOREVER);
		issued = udc_dwc3_depcmd_update_xfer(dev, wd_ep);
		udc_unlock_internal(dev);

		if (!issued) {
			/* Do not report or count a re-cache that never happened. */
			priv->ctrl_setup_wd_updxfer--;
			LOG_ERR("Update Transfer on EP%02x refused: no transfer "
				"resource index established", wd_ep->cfg.addr);
		} else {
		LOG_ERR("re-cached EP%02x (type 0x%02x) with Update Transfer "
			"(attempt %u): if the stage completes now, the controller was "
			"holding a stale descriptor",
			wd_ep->cfg.addr, priv->watchdog_type,
			priv->ctrl_setup_wd_updxfer);
		}

		k_work_reschedule(&priv->watchdog_dwork,
				  K_MSEC(CONFIG_UDC_DWC3_RECOVERY_TIMEOUT));
		return;
	}

	/*
	 * Escalate a stuck DATA or STATUS stage once Set Stall has had a whole
	 * episode to work and retired nothing.
	 */
	if (priv->watchdog_type != UDC_DWC3_WATCHDOG_TYPE_NONE &&
	    priv->watchdog_type != UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_SETUP) {
		const uint32_t stage_total =
			priv->ctrl_setup_done + priv->ctrl_status_done;

		if (priv->ctrl_recover_mark == stage_total) {
			priv->ctrl_setup_wd_reset++;
			udc_dwc3_setup_stuck_reset(dev);
			return;
		}

		priv->ctrl_recover_mark = stage_total;
	}

	/*
	 * Last check before the general recovery, same rule as the SETUP path
	 * above:
	 */
	{
		/* Under the mutex, for the reason given at the SETUP guard above. */
		bool machine_owns;

		udc_lock_internal(dev, K_FOREVER);
		machine_owns = udc_dwc3_recov_owns(&wd_cfg->ep_data_out[0]) ||
			       udc_dwc3_recov_owns(&wd_cfg->ep_data_in[0]);
		udc_unlock_internal(dev);

		if (machine_owns) {
			k_work_reschedule(&priv->watchdog_dwork,
					  K_MSEC(CONFIG_UDC_DWC3_RECOVERY_TIMEOUT));
			return;
		}
	}

	udc_dwc3_recover(dev);
}

/*
 * Upper bound on the wait for a posted event write to land in the buffer.
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
 */
BUILD_ASSERT(ARRAY_SIZE(((struct udc_dwc3_data *)0)->evt_copy) >=
	     CONFIG_UDC_DWC3_EVENTS_NUM,
	     "evt_copy must hold a full drain: udc_dwc3_evt_drain() clamps 'want' "
	     "to CONFIG_UDC_DWC3_EVENTS_NUM and indexes evt_copy by it");

/*
 * Two separate reasons, recorded because the databook does not give either and
 * reading it alone leads the wrong way: GEVNTSIZ.EVENTSIZ is a 16-bit byte
 * count, so the databook permits up to 64KB and says nothing about 64 bytes.
 */
BUILD_ASSERT(CONFIG_UDC_DWC3_EVENTS_NUM * sizeof(uint32_t) <= 64,
	     "DWC3 event ring is capped by the AXI block on this part, and "
	     "evt_copy - the local pre-credit copy - stays at 64 bytes");

/*
 * How many times to LOOK for the event word before giving up, rather than how
 * long to wait for it.
 */
#define UDC_DWC3_EVT_ARRIVE_FAST_POLLS 16u
/*
 * Wall-clock ceiling on the same wait, applied ALONGSIDE the poll count rather
 * than instead of it.
 */
#define UDC_DWC3_EVT_ARRIVE_MAX_MS 100u
/*
 * The control watchdog must outlast one full arrival wait, or it recovers a
 * drain that is still inside its normal budget.
 */
BUILD_ASSERT(CONFIG_UDC_DWC3_RECOVERY_TIMEOUT > UDC_DWC3_EVT_ARRIVE_MAX_MS,
	     "CONFIG_UDC_DWC3_RECOVERY_TIMEOUT must exceed "
	     "UDC_DWC3_EVT_ARRIVE_MAX_MS or the watchdog races the drain's own "
	     "arrival wait (build_flir's 50 ms setting does not)");
/*
 * heartbeat, nudge, watchdog and every endpoint's arm worker share one queue
 * thread, and that is what makes them mutually exclusive. Several cancel and
 * ordering arguments in this driver depend on it.
 */
BUILD_ASSERT(!IS_ENABLED(CONFIG_UDC_WORKQUEUE),
	     "this driver serialises its work items by sharing k_sys_work_q; "
	     "CONFIG_UDC_WORKQUEUE=y splits them and breaks that assumption");
/*
 * The slow phase: how long each timeout-based yield lasts, and how many of them.
 */
#define UDC_DWC3_EVT_SLOW_POLL_MS 10u
#define UDC_DWC3_EVT_ARRIVE_SLOW_POLLS 8u
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
 */
#define UDC_DWC3_EVT_FORCE_MIN_GAP_MS				500u

/*
 * UDC_DWC3_EVT_GAVEUP_RETRY_MS is gone with the block that used it. A stalled
 * pass no longer re-submits itself for a bounded window: the drain never arms
 * itself at all, and re-entry is udc_dwc3_drain_helper()'s decision on the next
 * heartbeat. The capture that justified the old window - a full ring sitting
 * 1.7-2.0 s with a valid event in the head slot and nothing looking at it -
 * is still the reason the helper kicks unconditionally on GEVNTCOUNT.
 */
#define UDC_DWC3_EVT_ARRIVE_POLL_US 1u
#define UDC_DWC3_EVT_GAVEUP_LOG_EVERY 1021u
/*
 * How often the running totals are reported without being asked for.
 */

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
	k_spinlock_key_t key;

	/*
	 * Only while the ring still has room. Every forced command lands an
	 * event of its own, and while the head slot is stuck that event queues
	 * BEHIND it.
	 */
	if (priv->evt_gc_last > UDC_DWC3_EVT_FORCE_MAX_GEVNTCOUNT) {
		return;
	}

	/*
	 * DGCMD bit 10 CMDACT, R/W1S: software sets it to start the generic command and
	 * the controller clears it when done.  Set therefore means the previous command
	 * is still executing.  The databook does not define writing over that, and this
	 * is not the place to find out - forces are milliseconds apart and a generic
	 * command retires in microseconds, so this should never be taken.
	 */
	key = k_spin_lock(&priv->dgcmd_lock);

	if (sys_read32(base + UDC_DWC3_DGCMD) & UDC_DWC3_DGCMD_ACT) {
		k_spin_unlock(&priv->dgcmd_lock, key);
		return;
	}

	sys_write32(0, base + UDC_DWC3_DGCMDPAR);
	sys_write32(UDC_DWC3_DGCMD_SET_PERIODIC_PARAMS | UDC_DWC3_DGCMD_IOC |
		    UDC_DWC3_DGCMD_ACT, base + UDC_DWC3_DGCMD);

	k_spin_unlock(&priv->dgcmd_lock, key);

	/*
	 * OUTSIDE the lock. CONFIG_LOG_MODE_MINIMAL is what the FLIR application
	 * builds with, and that backend formats and writes to the UART in the
	 * caller's context - this line is ~45 characters, about 3.9 ms at
	 * 115200.
	 */
	LOG_WRN_RATELIMIT("forced a generic command to unstick slot %u",
			  priv->evt_next);
}

/*
 * Issue the DGCMD the drain asked for. DGCMD must not be written from the
 * drain's context, so the request is handed here.
 */
static void udc_dwc3_nudge_worker(struct k_work *const work)
{
	struct udc_dwc3_data *const priv =
		CONTAINER_OF(work, struct udc_dwc3_data, nudge_work);
	const struct device *const dev = priv->dev;

	if (k_cyc_to_ms_near32(k_cycle_get_32() - priv->evt_force_t0) <
					UDC_DWC3_EVT_FORCE_MIN_GAP_MS) {
		return;
	}

	udc_dwc3_evt_force(dev);
	priv->evt_force_t0 = k_cycle_get_32();
}

/*
 * The heartbeat's ENTIRE involvement with the event ring: restart a drain that
 * has stopped while the controller still owes events. That is all that is left
 * here, and it is all that was ever safe to do from this context.
 */
static void udc_dwc3_drain_helper(const struct device *const dev)
{
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

	/*
	 * THE HEARTBEAT'S ONE READ, and it publishes what it read: the heartbeat
	 * worker and udc_dwc3_evt_force() consume priv->evt_gc_last rather than
	 * touching the register.
	 */
	priv->evt_gc_last = udc_dwc3_gevntcount(base);

	if (priv->evt_gc_last == 0U) {
		return;
	}

	/*
	 * One state IS read, and only to answer "is this thread already awake
	 * and working".
	 */
	if (priv->drain.state == UDC_DWC3_DRAIN_WAITING) {
		return;
	}

	if (k_cyc_to_ms_near32(k_cycle_get_32() - priv->evt_worker_exit_t0) >=
					UDC_DWC3_EVT_IDLE_KICK_MS) {
		priv->evt_kick++;
		k_sem_give(&priv->evt_sem);
	}
}

/*
 * Wait for the FIRST word of a pass, which is the only one worth waiting for:
 * there is nothing copied yet, so returning without it would just spin.
 */
static uint32_t udc_dwc3_evt_wait_first(const struct device *const dev,
					const uint32_t gc)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const uint32_t t0 = k_cycle_get_32();
	const uint32_t deadline = t0 + k_ms_to_cyc_ceil32(UDC_DWC3_EVT_ARRIVE_MAX_MS);
	uint32_t polls = 0;
	uint32_t slow_polls = 0;
	uint32_t evt;
	uint32_t waited_us;
	bool new_run = false;

	priv->evt_late++;
	priv->drain.state = UDC_DWC3_DRAIN_WAITING;

	/*
	 * LOOK BEFORE WAITING. The caller read this slot as unwritten and then
	 * spent a call frame, a counter and a state write getting here, and the
	 * controller may well have filled it in between - a do/while spends a full
	 * poll interval before it will even look, so an event that is already in
	 * memory is reported as late and costs UDC_DWC3_EVT_ARRIVE_POLL_US for
	 * nothing. Reading first costs one load.
	 */
	/* PHASE 1, the microsecond instrument: a bounded spin, no reschedule. */
	for (;;) {
		evt = cfg->evt_buf[priv->evt_next];
		if (evt != UDC_DWC3_EVT_CONSUMED_ENTRY_VALUE) {
			break;
		}

		if (polls >= UDC_DWC3_EVT_ARRIVE_FAST_POLLS) {
			break;
		}

		/* The deadline binds this phase too, not only the slow one. */
		if ((int32_t)(k_cycle_get_32() - deadline) >= 0) {
			break;
		}

		k_busy_wait(UDC_DWC3_EVT_ARRIVE_POLL_US);
		polls++;
	}

	/* PHASE 2, the long tail: a TIMEOUT-BASED sleep, not a bare k_yield(). */
	while (evt == UDC_DWC3_EVT_CONSUMED_ENTRY_VALUE &&
	       slow_polls < UDC_DWC3_EVT_ARRIVE_SLOW_POLLS) {
		/*
		 * Signed difference, so the comparison survives the 32-bit cycle
		 * counter wrapping inside the loop. See UDC_DWC3_EVT_ARRIVE_MAX_MS.
		 */
		if ((int32_t)(k_cycle_get_32() - deadline) >= 0) {
			break;
		}

		k_sleep(K_MSEC(UDC_DWC3_EVT_SLOW_POLL_MS));
		slow_polls++;

		evt = cfg->evt_buf[priv->evt_next];
	}

	waited_us = k_cyc_to_us_near32(k_cycle_get_32() - t0);

	if (evt != UDC_DWC3_EVT_CONSUMED_ENTRY_VALUE) {
		/*
		 * FAST polls only, deliberately. This counter and
		 * evt_late_us_max are read as a pair - "worst %u polls/%u us" -
		 * and the pair only means anything while both come from the same
		 * clock.
		 */
		if (polls > priv->evt_late_polls_max) {
			priv->evt_late_polls_max = polls;
		}

		if (waited_us > priv->evt_late_us_max) {
			priv->evt_late_us_max = waited_us;
		}

		/* The write landed: the episode is over, back to ordinary draining. */
		priv->drain.state = UDC_DWC3_DRAIN_RUNNING;

		return evt;
	}

	priv->evt_gaveup++;

	if (priv->drain.attempts > 0 && priv->drain.slot == priv->evt_next) {
		priv->drain.attempts++;
	} else {
		new_run = true;
		priv->drain.slot = priv->evt_next;
		priv->drain.attempts = 1;
		priv->drain.since = t0;
		priv->drain.quiet = true;
		priv->drain.counted = false;
		/* The pass's one read, not a new one. */
		priv->drain.gc0 = gc;

		/*
		 * Population data for the look-ahead in
		 * udc_dwc3_evt_skip_dead_slot():
		 */
		if (priv->drain.gc0 > priv->evt_gaveup_gc0_max) {
			priv->evt_gaveup_gc0_max = priv->drain.gc0;
		}
		if (priv->drain.gc0 > sizeof(uint32_t)) {
			priv->evt_gaveup_multi++;
		}
	}

	/*
	 * Say once, per run, that this is a LOST write rather than a late one -
	 * and say which kind, because the two need opposite responses. the
	 * databook's "events are queued up internally...
	 */
	if (!priv->drain.counted &&
	    k_cyc_to_ms_near32(k_cycle_get_32() - priv->drain.since) >=
						UDC_DWC3_EVT_MISSED_MS) {
		/*
		 * A CROSS-PASS COMPARISON, NOT A SECOND READ: gc is this pass's
		 * single read, drain.gc0 the single read of the pass that opened
		 * this run. Re-reading here would race the controller's writes.
		 */
		const uint32_t gc_now = gc;
		const bool frozen = (gc_now == priv->drain.gc0);

		priv->drain.counted = true;
		priv->evt_missed++;
		if (frozen) {
			priv->evt_missed_frozen++;
		}

		priv->drain.quiet = false;
		LOG_ERR("slot %u WRITE LOST, not late: empty %u ms over %u give-ups, "
			"GEVNTCOUNT %s (%u B now, %u B when the run opened) - %s",
			priv->evt_next,
			k_cyc_to_ms_near32(k_cycle_get_32() - priv->drain.since),
			priv->drain.attempts,
			frozen ? "FROZEN" : "advancing",
			gc_now, priv->drain.gc0,
			frozen ? "core has placed nothing since; cause unknown"
			       : "core still writing, this slot skipped");

		/*
		 * NO REGISTER DUMP HERE. This runs inside
		 * udc_dwc3_evt_wait_first(), which runs inside the drain:
		 */
	}

	/*
	 * Anything emitted from here on sits INSIDE the window that drain.since
	 * is timing, and at 115200 baud one of these lines is 7.5 ms - larger
	 * than the latency being measured.
	 */
	if (priv->evt_gaveup % UDC_DWC3_EVT_GAVEUP_LOG_EVERY == 1) {
		priv->drain.quiet = false;
		LOG_DBG("slot %u empty: polls=%u+%u slow waited=%uus gc %u B "
			"hwm=%u run=%u (%u so far)",
			priv->evt_next, polls, slow_polls, waited_us, gc,
			priv->evt_gevntcount_hwm, priv->drain.attempts,
			priv->evt_gaveup);
	}

	/*
	 * Force at the first discovery of a stall, then no more often than
	 * UDC_DWC3_EVT_FORCE_MIN_GAP_MS.
	 */
	/* Request only; udc_dwc3_nudge_worker() issues the DGCMD. */
	priv->drain.state = UDC_DWC3_DRAIN_NUDGE;

	if (new_run) {
		k_work_submit_to_queue(udc_get_work_q(), &priv->nudge_work);
	}

	/*
	 * drain.quiet marks a give-up run as unmeasured, and is cleared ONLY by
	 * the two sites that print from this thread - the WRITE LOST report and
	 * the 1-in-1021 debug line.
	 */

	/* Nothing landed: report it the same way an unwritten slot reads. */
	return UDC_DWC3_EVT_CONSUMED_ENTRY_VALUE;
}

/*
 * Drain the ring into the drain FIFO and hand every slot back in ONE acknowledge,
 * before a single event is dispatched.
 *
 * A single acknowledge of everything copied is also what the databook requires to
 * escape an overflow: "software must free up space in the Event Buffer by
 * acknowledging more than 1 event".
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
 */
/*
 * Walk every endpoint and retire what the TRB rings say is already finished.
 */
static void udc_dwc3_evt_reconcile_endpoints(const struct device *const dev)
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
			const uint8_t why = udc_dwc3_recov_observe(dev, &eps[i]);

			if (why == UDC_DWC3_RECOV_R_UNPOPPED) {
				rescued += udc_dwc3_drain_completed(dev, &eps[i]);
			} else if (why == UDC_DWC3_RECOV_R_NO_SETUP) {
				/*
				 * EP0 IS WHERE THIS DEVICE ACTUALLY DIES, and
				 * the first version of this reconcile walked
				 * straight past it:
				 */
				ctrl_rearmed = true;
				udc_dwc3_ctrl_reset_to_step1(dev, false);
				udc_dwc3_ctrl_next(dev);
			}
		}
	}

	udc_unlock_internal(dev);

	if (rescued > 0U || ctrl_rearmed) {
		priv->evt_sweep_rescued += rescued;
		priv->evt_sweep_runs++;
		LOG_WRN("discard reconciled: retired %u completion(s)%s (%u buffers "
			"over %u discards)", rescued,
			ctrl_rearmed ? " and re-armed a stopped control stage" : "",
			priv->evt_sweep_rescued, priv->evt_sweep_runs);
	}
}

/*
 * Decide how many event slots the controller will never fill, and step over
 * them. Returns the number stepped over; the CALLER folds that into the single
 * GEVNTCOUNT credit its pass writes.
 *
 * This function touches no register. GEVNTCOUNT is a decrement-on-write counter
 * the controller updates concurrently, so "write the credit, read it back,
 * decide whether it landed" cannot work: a concurrent event write and a refused
 * credit produce the same observation. Because the pass settles everything it
 * consumed before it returns, nothing is ever outstanding and nothing needs
 * verifying.
 */
static uint32_t udc_dwc3_evt_skip_dead_slot(const struct device *const dev,
					const uint32_t gc, const bool frozen,
					const uint32_t gaveup_ms)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_data *const priv = udc_get_private(dev);

	/* The caller decides; this only acts. */
	/* How far to advance. */
	uint32_t owed = gc / sizeof(uint32_t);
	uint32_t skip = 1u;

	if (owed > (CONFIG_UDC_DWC3_EVENTS_NUM - 1u)) {
		owed = CONFIG_UDC_DWC3_EVENTS_NUM - 1u;
	}

	/*
	 * A WRITTEN SLOT IS THE ONLY THING THAT EXTENDS THE SKIP. Finding one
	 * proves the controller passed over the j slots before it. Finding none
	 * proves nothing - those words may still be in flight - so step over one.
	 */
	for (uint32_t j = 1u; j < owed; j++) {
		const uint32_t idx =
			(priv->evt_next + j) % CONFIG_UDC_DWC3_EVENTS_NUM;

		if (cfg->evt_buf[idx] != UDC_DWC3_EVT_CONSUMED_ENTRY_VALUE) {
			skip = j;
			break;
		}
	}

	LOG_ERR("slot %u unreadable for %u ms over %u give-ups (holds 0x%08x, "
		"gc %u B, %s): acknowledging %u slot%s to release the "
		"controller, %u EVENT%s LOST (%u so far)",
		priv->evt_next, gaveup_ms, priv->drain.attempts,
		cfg->evt_buf[priv->evt_next], gc,
		frozen ? "GEVNTCOUNT frozen - core blocked on us"
		       : "GEVNTCOUNT advancing - core still writing elsewhere",
		skip, skip == 1u ? "" : "s",
		skip, skip == 1u ? "" : "S",
		priv->evt_skipped + skip);

	for (uint32_t j = 0u; j < skip; j++) {
		cfg->evt_buf[(priv->evt_next + j) % CONFIG_UDC_DWC3_EVENTS_NUM] =
			UDC_DWC3_EVT_CONSUMED_ENTRY_VALUE;
	}
	priv->evt_next = (priv->evt_next + skip) % CONFIG_UDC_DWC3_EVENTS_NUM;
	priv->evt_skipped += skip;
	priv->drain.attempts = 0;

	return skip;
}

/*
 * Is the slot the drain is parked on dead?
 */
static bool udc_dwc3_drain_slot_is_dead(const struct device *const dev,
					const uint32_t gc, const uint32_t age_ms)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_data *const priv = udc_get_private(dev);

	if (gc < sizeof(uint32_t) ||
	    cfg->evt_buf[priv->evt_next] != UDC_DWC3_EVT_CONSUMED_ENTRY_VALUE) {
		return false;
	}

	/*
	 * THE PROOF FIRST, AND ON ITS OWN FLOOR.
	 *
	 * A written slot BEYOND this one proves the head event is gone: the
	 * controller fills the ring in order, so having placed a later entry it
	 * has passed this one by. That is evidence, not a timeout, and it must
	 * not be gated behind the timeout floor - a single floor applied to
	 * every route made UDC_DWC3_EVT_LOOKAHEAD_MIN_MS dead and delayed a
	 * PROVEN loss by four times its own budget, during which the ring does
	 * not drain and the fault compounds.
	 *
	 * Note what this deliberately does NOT do: GEVNTCOUNT alone is never
	 * enough. gc = 8 with both slots still unwritten means two events were
	 * COUNTED and neither has landed - they may be late, not lost. Only a
	 * written slot ahead distinguishes the two.
	 */
	if (age_ms >= UDC_DWC3_EVT_LOOKAHEAD_MIN_MS &&
	    udc_dwc3_evt_lookahead_lost(dev, gc)) {
		priv->evt_lookahead_short++;
		return true;
	}

	/* No proof available - fall back on the timeout routes, with the floor. */
	if (age_ms < UDC_DWC3_EVT_DEAD_SLOT_MIN_MS) {
		return false;
	}

	if (age_ms >= UDC_DWC3_EVT_DEAD_SLOT_MS ||
	    priv->drain.attempts >= UDC_DWC3_EVT_DEAD_SLOT_GIVEUPS) {
		return true;
	}

	return false;
}

/*
 * Take every event the controller has written, dispatch it, and credit
 * GEVNTCOUNT. Returns how many were handled.
 */
static uint32_t udc_dwc3_evt_drain(const struct device *const dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	/*
	 * The one and only read of GEVNTCOUNT for this pass, taken before any
	 * event is processed, which is exactly what section 1.2.56 requires -
	 * see the clock-crossing quotation at UDC_DWC3_GEVNTCOUNT_MASK.
	 */
	const uint32_t gc = udc_dwc3_gevntcount(base);
	uint32_t want = gc / sizeof(uint32_t);
	uint32_t n = 0;
	/*
	 * Dead slots stepped over in this pass. They carry no event to dispatch
	 * but they are words the controller is owed an acknowledgement for, so
	 * they go into the ONE credit written at the end of the pass.
	 */
	uint32_t skipped = 0;

	/*
	 * NUDGE SURVIVES A RE-ENTRY; everything else is re-derived from gc. This
	 * assignment used to be unconditional, which quietly made NUDGE
	 * unobservable. A stuck head slot re-enters this function at interrupt
	 * rate (the zero-credit write clears EVNT_HANDLER_BUSY, the end of the
	 * pass unmasks the interrupt, and GEVNTCOUNT is still non-zero, so the
	 * controller re-asserts immediately), so the state set at the end of one
	 * pass was overwritten microseconds later at the top of the next.
	 */
	if (priv->drain.state != UDC_DWC3_DRAIN_NUDGE) {
		priv->drain.state = (gc > 0U) ? UDC_DWC3_DRAIN_RUNNING
					      : UDC_DWC3_DRAIN_IDLE;
	}

	/* Published for every other reader; nobody else touches the register. */
	priv->evt_gc_last = gc;

	if (gc > priv->evt_gevntcount_hwm) {
		priv->evt_gevntcount_hwm = gc;
	}

	/* Hard bound before indexing. The count comes from a register; */
	if (want > CONFIG_UDC_DWC3_EVENTS_NUM) {
		LOG_ERR_RATELIMIT("GEVNTCOUNT reports %u B, more than the %u B ring",
				  gc, (unsigned int)(CONFIG_UDC_DWC3_EVENTS_NUM *
						     sizeof(uint32_t)));
		want = CONFIG_UDC_DWC3_EVENTS_NUM;
	}

	while (n < want) {
		uint32_t evt = cfg->evt_buf[priv->evt_next];

		if (evt == UDC_DWC3_EVT_CONSUMED_ENTRY_VALUE) {
			/* Mid-pass: do not wait. */
			if (n > 0) {
				priv->evt_midzero++;
				priv->drain.state = UDC_DWC3_DRAIN_PARTIAL;
				break;
			}

			evt = udc_dwc3_evt_wait_first(dev, gc);
			if (evt == UDC_DWC3_EVT_CONSUMED_ENTRY_VALUE) {
				/*
				 * Decided and acted on HERE, by the owner of
				 * evt_next. wait_first() has left the state at
				 * NUDGE, which is what the heartbeat reads to
				 * decide whether to send a generic command.
				 */
				/* gc is the pass's one read, not a new one. */
				const uint32_t age_ms = k_cyc_to_ms_near32(
					k_cycle_get_32() - priv->drain.since);

				if (udc_dwc3_drain_slot_is_dead(dev, gc, age_ms)) {
#ifdef UDC_DWC3_EVT_DEAD_SLOT_RECOVER
					/*
					 * THE RESULT IS NOT OPTIONAL. This was a
					 * (void) cast, so the two opposite
					 * outcomes - the controller took the
					 * credit, and the controller did not -
					 * both fell through to the same code below
					 * and were reported as a discard that had
					 * happened.
					 */
					/*
					 * FOLDED INTO THIS PASS'S ONE CREDIT.
					 * Skipping cannot fail: it is a decision
					 * taken from the ring contents, not a
					 * write whose fate must be discovered.
					 */
					skipped += udc_dwc3_evt_skip_dead_slot(
						dev, gc,
						gc == priv->drain.gc0, age_ms);

					udc_dwc3_recov_note_discard(dev);
					udc_dwc3_evt_reconcile_endpoints(dev);
					priv->drain.state = UDC_DWC3_DRAIN_RUNNING;
#else
					/*
					 * Diagnostic build: describe it and leave
					 * the ring, GEVNTCOUNT and the controller
					 * exactly as the fault left them, so it
					 * stays reproducible.
					 */
					LOG_ERR("slot %u is dead (%u ms, %u "
						"give-ups, gc %u B, %s) - NOT "
						"recovering, this build preserves "
						"the fault",
						priv->evt_next, age_ms,
						priv->drain.attempts, gc,
						gc == priv->drain.gc0 ?
							"GEVNTCOUNT frozen"
						      : "GEVNTCOUNT advancing");
#endif
				}
				break;
			}
		}

		if (priv->drain.attempts > 0 &&
		    priv->drain.slot == priv->evt_next) {
			/*
			 * How long the slot actually stayed empty - the number
			 * the RTL question turns on, so it is taken only from
			 * runs that printed nothing while it was being timed.
			 */
			if (priv->drain.quiet) {
				const uint32_t us = k_cyc_to_us_near32(
					k_cycle_get_32() - priv->drain.since);

				if (us > priv->evt_gaveup_us_max) {
					priv->evt_gaveup_us_max = us;
				}
			} else {
				LOG_DBG("slot %u filled after %u give-ups: 0x%08x (%s) "
					"gc0 %u B - interval not timed, this run printed",
					priv->evt_next, priv->drain.attempts, evt,
					udc_dwc3_get_event_name(evt,
						sys_read32(base + UDC_DWC3_DSTS)),
					priv->drain.gc0);
			}
			priv->drain.attempts = 0;

			/*
			 * THE EPISODE IS OVER, so end the nudge with it. This is
			 * the exit that udc_dwc3_evt_wait_first() cannot take.
			 * Once the write lands, the head slot reads as a real
			 * event and wait_first() is never called again for it -
			 * so the RUNNING it writes on its own success path is
			 * unreachable here, and without this the latch
			 * introduced at the top of this function would hold
			 * NUDGE for the rest of the session, re-submitting
			 * nothing but reporting a permanent stall in every dump.
			 */
			priv->drain.state = UDC_DWC3_DRAIN_RUNNING;
		}

		/* A zero word here is DATA, not absence - and that is new. */
		if (evt == 0U) {
			priv->evt_zero++;
			LOG_DBG("evtword=0 at slot %u (%u so far): the "
				"controller wrote a zero over the free marker - "
				"this is a written word, not a missing one; "
				"gc %u B, next slot 0x%08x",
				priv->evt_next, priv->evt_zero,
				gc,
				cfg->evt_buf[(priv->evt_next + 1) %
					     CONFIG_UDC_DWC3_EVENTS_NUM]);
		}

		priv->evt_copy[n++] = evt;

		/*
		 * Re-arm BEFORE the acknowledge below. The slot belongs to
		 * software only until then;
		 */
		cfg->evt_buf[priv->evt_next] = UDC_DWC3_EVT_CONSUMED_ENTRY_VALUE;
		priv->evt_next = (priv->evt_next + 1) % CONFIG_UDC_DWC3_EVENTS_NUM;
	}

	/*
	 * THE ONE WRITE OF THIS PASS, carrying every word consumed: events read
	 * out plus dead slots stepped over. Zero is a legal, meaningful count -
	 * it credits nothing and clears EVNT_HANDLER_BUSY - so there is no
	 * branch here and no second write anywhere in the drain thread.
	 */
	udc_dwc3_gevntcount_ack(base, n + skipped);

	/*
	 * Close the state out. A pass that took everything is no longer running,
	 * and leaving RUNNING behind is exactly the stale-state bug this enum
	 * exists to prevent - it would have the dump reporting a drain that is
	 * working while it sits idle.
	 */
	if ((priv->drain.state == UDC_DWC3_DRAIN_RUNNING ||
	     priv->drain.state == UDC_DWC3_DRAIN_NUDGE) &&
	    (n + skipped) >= want) {
		priv->drain.state = UDC_DWC3_DRAIN_IDLE;
	}

	return n;
}

static void udc_dwc3_event_drain_once(const struct device *const dev);

/*
 * The event ring gets its own thread.
 *
 * It used to be a work item on udc_get_work_q(), sharing that single queue
 * thread with ep_data->work and heartbeat_work. The drain itself needs no
 * mutex - udc_dwc3_evt_drain() reads the ring and credits GEVNTCOUNT and that
 * is all - but the dispatch that follows it takes the UDC mutex, and a queue
 * runs its items one at a time. So a dispatch blocked on the mutex stopped the
 * worker returning, which stopped the next drain, which is the one thing
 * section 3.2.2.5 says must never stop: "Software must always service the event
 * interrupts generated by the controller."
 */
static void udc_dwc3_event_thread(void *const p1, void *const p2, void *const p3)
{
	struct udc_dwc3_data *const priv = p1;
	const struct device *const dev = priv->dev;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		k_sem_take(&priv->evt_sem, K_FOREVER);
		udc_dwc3_event_drain_once(dev);
	}
}

/*
 * One pass of the event drain, timestamped for the liveness check.
 */
static void udc_dwc3_event_drain_once(const struct device *const dev)
{
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const struct udc_dwc3_config *const cfg = dev->config;
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	const uint32_t n = (priv->evt_worker_runs++, udc_dwc3_evt_drain(dev));

	/* The ring is already back with the controller by this point. */
	for (uint32_t i = 0; i < n; i++) {
		udc_dwc3_handle_event(dev, priv->evt_copy[i]);

		priv->evt_handled++;

	}

	/* IRQ-LOCKED, because the other writer of this bit is the ISR. */
	{
		const unsigned int key = irq_lock();

		sys_clear_bits(base + UDC_DWC3_GEVNTSIZ(0),
			       UDC_DWC3_GEVNTSIZ_EVNTINTRPTMASK);
		irq_unlock(key);
	}
	cfg->irq_enable_func();

	/* The controller still owes events, so this pass has to be re-entered. */


	/*
	 * Last thing done, so this records when the pass FINISHED - see
	 * UDC_DWC3_EVT_IDLE_KICK_MS for why the exit and not the entry.
	 */
	priv->evt_worker_exit_t0 = k_cycle_get_32();
}

/*
 * Event interrupt: mask, and wake the drain thread.
 */
static void udc_dwc3_irq_handler(void *const ptr)
{
	const struct device *const dev = ptr;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const struct udc_dwc3_config *const cfg = dev->config;
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

	priv->evt_isr++;

	k_sem_give(&priv->evt_sem);

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

/*
 * UDC API: cancel queued buffers on an endpoint.
 */
static int udc_dwc3_ep_dequeue(const struct device *const dev,
			       struct udc_ep_config *const ep_cfg)
{
	struct udc_dwc3_ep_data *const ep_data =
		CONTAINER_OF(ep_cfg, struct udc_dwc3_ep_data, cfg);

	/*
	 * SPEC, Programming Guide 3.30b: "it is recommended that software issue an
	 * End Transfer command for the endpoint/transfer resource before
	 * de-allocating the memory."
	 */
	/* NOT on the control endpoints. */
	if (USB_EP_GET_IDX(ep_cfg->addr) != 0U &&
	    udc_dwc3_ep_ring_outstanding(ep_data) &&
	    udc_dwc3_depcmd_end_xfer(dev, ep_data, UDC_DWC3_DEPCMD_HIPRI_FORCERM)) {
		LOG_INF("EP%02x dequeued with a transfer still active; recovering "
			"through End Transfer before the buffers are released",
			ep_cfg->addr);
	}

	/*
	 * RETURN THE BUFFERS THE DRIVER HOLDS, NOT JUST THE STACK'S QUEUE.
	 *
	 * udc_ep_cancel_queued() drains cfg->fifo only. Buffers already moved
	 * into ep_data->net_buf[] by the arm path are invisible to it, and the
	 * End Transfer above has just stopped the controller, so they would
	 * never be retired either - each dequeue would lose up to
	 * CONFIG_UDC_DWC3_TRB_NUM - 1 buffers from a fixed pool, permanently.
	 *
	 * ep_ring_release() parks them on requeue_fifo; unlike a disable, which
	 * parks them deliberately so udc_dwc3_ep_resume() can re-arm them across
	 * an alt-setting switch, a dequeue means CANCEL, so they go back to the
	 * stack with -ECONNABORTED.
	 */
	if (USB_EP_GET_IDX(ep_cfg->addr) != 0U) {
		udc_dwc3_ep_ring_release(ep_data);
		udc_dwc3_ep_return_parked(dev, ep_data, -ECONNABORTED);
	}

	udc_ep_cancel_queued(dev, ep_cfg);
	udc_ep_set_busy(ep_cfg, false);

	return 0;
}

/*
 * Re-establish an endpoint: configure it, enable it in DALEPENA and arm whatever
 * is queued. modify selects DEPCFG Modify over Init.
 */
static int udc_dwc3_ep_resume(const struct device *const dev,
			      struct udc_dwc3_ep_data *const ep_data,
			      const bool modify)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	struct net_buf *buf;
	int ret;

	/*
	 * Databook 3.2.2.7: a Start Transfer must not be issued on an endpoint
	 * whose End Transfer has not reported Endpoint Command Complete.
	 */
	if (USB_EP_GET_IDX(ep_data->cfg.addr) > 0 && udc_dwc3_ep_is_ending(ep_data)) {
		LOG_DBG("End Transfer still concluding on EP%02x, deferring resume",
			ep_data->cfg.addr);
		ep_data->resume_modify = modify;
		(void)udc_dwc3_ep_state_set(ep_data, UDC_DWC3_EP_ENDING_RESUME);
		return 0;
	}

	/* Reset all ongoing transfers on non-control OUT endpoints */
	if (USB_EP_GET_IDX(ep_data->cfg.addr) > 0) {
		udc_dwc3_depcmd_clear_stall(dev, ep_data, UDC_DWC3_DEPCMD_HIPRI_FORCERM);
	}

	udc_dwc3_depcmd_ep_config(dev, ep_data, modify);

	/*
	 * INVARIANT 4: DEPXFERCFG only on enable, NEVER on a resume - AND ONLY
	 * ONCE PER POOL GENERATION.
	 *
	 * `modify` alone cannot carry this. udc_ep_enable_internal() calls
	 * api->ep_enable() while cfg.stat.enabled is still false and only sets it
	 * afterwards, so udc_dwc3_ep_enable() always passes modify = false. An
	 * alt-setting switch is a disable/enable pair, and the re-enable of an
	 * endpoint that is not first_ep runs no DEPSTARTCFG - so without this
	 * test each switch allocated one more transfer resource for that
	 * endpoint, none of which End Transfer gives back, until Start Transfer
	 * answered CmdStatus 4'h1 for good.
	 */
	if (!modify && ep_data->xfercfg_epoch != priv->xfercfg_epoch) {
		udc_dwc3_depcmd_ep_xfer_config(dev, ep_data);
		ep_data->xfercfg_epoch = priv->xfercfg_epoch;
	}

	if (USB_EP_GET_IDX(ep_data->cfg.addr) > 0) {
		ret = udc_dwc3_trb_nonctrl_init(dev, ep_data);
		if (ret != 0) {
			return ret;
		}
	}

	/* Starting from here, the endpoint can be used */
	sys_set_bits(base + UDC_DWC3_DALEPENA, UDC_DWC3_DALEPENA_USBACTEP(ep_data->epn));

	/* Re-enqueue all the previously dequeued buffers */
	/*
	 * Peek, arm, then remove - the order udc_dwc3_ep_worker() uses on the
	 * stack queue, for the same reason.
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
static int udc_dwc3_ep_enable(const struct device *const dev, struct udc_ep_config *const ep_cfg)
{
	struct udc_dwc3_ep_data *const ep_data = (struct udc_dwc3_ep_data *)ep_cfg;
	struct udc_dwc3_data *const priv = udc_get_private(dev);

	LOG_DBG("EP%02x, first EP%02x", ep_data->cfg.addr, priv->first_ep);

	/*
	 * REFUSE ISOCHRONOUS RATHER THAN MIS-ARM IT - see caps.iso above. The
	 * capability is not advertised, so this should be unreachable; it is the
	 * backstop for a class that configures one anyway.
	 */
	if ((ep_cfg->attributes & USB_EP_TRANSFER_TYPE_MASK) ==
	    USB_EP_TYPE_ISO) {
		LOG_ERR("EP%02x isochronous is not supported by this driver",
			ep_cfg->addr);
		return -ENOTSUP;
	}

	if (USB_EP_GET_IDX(ep_cfg->addr) > 0) {
		if (priv->first_ep == 0) {
			priv->first_ep = ep_cfg->addr;
		}
		if (ep_cfg->addr == priv->first_ep && !priv->cfg_pool_assigned) {
			priv->cfg_pool_assigned = true;
			udc_dwc3_on_set_config_or_interface(dev);
		}
	}

	return udc_dwc3_ep_resume(dev, ep_data, ep_data->cfg.stat.enabled);
}

/*
 * UDC API: disable an endpoint. Returns -EBUSY if the controller may still own
 * its buffers, in which case nothing is released.
 */
static int udc_dwc3_ep_disable(const struct device *const dev, struct udc_ep_config *const ep_cfg)
{
	struct udc_dwc3_ep_data *ep_data = CONTAINER_OF(ep_cfg, struct udc_dwc3_ep_data, cfg);
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

	LOG_DBG("Disabling EP%02x", ep_cfg->addr);

	/*
	 * THE STATE IS NOT CLEARED HERE, and that is the whole point. This used
	 * to zero xferrscidx at the top of the disable. The End Transfer below
	 * then found no index and refused - udc_dwc3_depcmd_end_xfer() has
	 * always declined on INVALID - so every endpoint disable tore the
	 * endpoint down WITHOUT ending its transfer, and the controller never
	 * got the transfer resource back.
	 */

	/*
	 * Drop any reference the control machinery holds to this endpoint before
	 * tearing it down.
	 */
	if (priv->watchdog_ep == ep_data) {
		k_work_cancel_delayable(&priv->watchdog_dwork);
		priv->watchdog_ep = NULL;
		priv->watchdog_type = UDC_DWC3_WATCHDOG_TYPE_NONE;
	}

	/*
	 * Drop any postponed resume as well. Its completion would otherwise run a
	 * resume left over from before this teardown and re-arm an endpoint that
	 * has just been disabled. The success path below reaches
	 * udc_dwc3_ep_state_reset() and drops it incidentally, but the -EBUSY
	 * return does not, so withdraw it here where every path passes.
	 */
	if (ep_data->xfer_state == UDC_DWC3_EP_ENDING_RESUME) {
		(void)udc_dwc3_ep_state_set(ep_data, UDC_DWC3_EP_ENDING);
	}

	/* Disable the endpoint */

	sys_clear_bits(base + UDC_DWC3_DALEPENA, UDC_DWC3_DALEPENA_USBACTEP(ep_data->epn));

	/* Reset ongoing transfers. */
	udc_dwc3_depcmd_end_xfer(dev, ep_data, UDC_DWC3_DEPCMD_HIPRI_FORCERM);

	/*
	 * F1: DO NOT RELEASE THE RING UNTIL THE CONTROLLER HAS FINISHED WITH IT.
	 * SPEC 3.2.2.7:
	 */
	if ((sys_read32(base + UDC_DWC3_DCTL) & UDC_DWC3_DCTL_RUNSTOP) != 0) {
		uint32_t done = 0;

		if (!udc_dwc3_wait_cmdact_zero(dev, UDC_DWC3_DEPCMD(ep_data->epn),
					       &done)) {
			/*
			 * DO NOT RELEASE THE RING. This used to warn that
			 * "buffers may still be referenced" and then release
			 * them anyway, which is the warning describing a bug
			 * rather than preventing one. CMDACT still set means the
			 * End Transfer has not finished and the controller may
			 * still be reading or writing these buffers;
			 */
			LOG_ERR("EP%02x End Transfer still active (0x%08x): ring NOT "
				"released and buffers NOT returned - the controller may "
				"still own them", ep_cfg->addr, done);
			udc_ep_set_busy(ep_cfg, false);
			return -EBUSY;
		}
	}

	udc_ep_set_busy(ep_cfg, false);

	/* Oldest first, back onto the requeue FIFO, and reset the ring. */
	udc_dwc3_ep_ring_release(ep_data);

	/*
	 * The endpoint is going away, so whatever the controller was doing with
	 * it is over.
	 */
	udc_dwc3_ep_state_reset(ep_data);

	return 0;
}

/*
 * UDC API: STALL an endpoint. cfg.stat.halted follows the hardware.
 */
static int udc_dwc3_ep_set_halt(const struct device *const dev,
				struct udc_ep_config *const ep_cfg)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	struct udc_dwc3_ep_data *ep_data = CONTAINER_OF(ep_cfg, struct udc_dwc3_ep_data, cfg);

	/*
	 * Say WHO halted the endpoint. A halt on a data endpoint is the visible
	 * start of a failure that ends somewhere else entirely - a capture had
	 * EP85 halted, the host clearing it 280 s later, and control stopping dead
	 * immediately after - and without this line there is no way to tell a halt
	 * this driver was asked for from one the controller raised on its own.
	 */
	LOG_INF("Set halt on EP%02x (requested by the stack)", ep_cfg->addr);

	switch (ep_data->cfg.addr) {
	case USB_CONTROL_EP_IN:
		/* The datasheet says to only set stall the OUT direction */
		ep_data = &cfg->ep_data_out[0];
		__fallthrough;
	case USB_CONTROL_EP_OUT:
		if (!udc_dwc3_depcmd_set_stall(dev, ep_data)) {
			LOG_ERR("EP%02x Set Stall was refused by the controller; "
				"reporting the failure rather than claiming the "
				"endpoint is halted", ep_data->cfg.addr);
			return -EIO;
		}
		break;
	default:
		/*
		 * cfg.stat.halted is only written after the hardware has
		 * actually taken the command.
		 */
		if (!udc_dwc3_depcmd_set_stall(dev, ep_data)) {
			LOG_ERR("EP%02x Set Stall was refused by the controller; "
				"endpoint is NOT halted", ep_data->cfg.addr);
			return -EIO;
		}
		ep_data->cfg.stat.halted = true;
		priv->ep_halts++;
	}

	/* So that the next type is SETUP */
	priv->last_xfer_type = UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_2;

	return 0;
}

/*
 * UDC API: clear an endpoint STALL. cfg.stat.halted follows the hardware.
 */
static int udc_dwc3_ep_clear_halt(const struct device *const dev,
				  struct udc_ep_config *const ep_cfg)
{
	struct udc_dwc3_ep_data *const ep_data = CONTAINER_OF(ep_cfg, struct udc_dwc3_ep_data, cfg);

	LOG_INF("Clearing stall for EP%02x", ep_cfg->addr);

	if (USB_EP_GET_IDX(ep_data->cfg.addr) == 0) {
		return 0;
	}

	/* SPEC, Programming Guide 3.30b section 4.2.7 "Handling ENDPOINT_HALT": */
	if (udc_dwc3_ep_ring_outstanding(ep_data) &&
	    udc_dwc3_depcmd_end_xfer(dev, ep_data, UDC_DWC3_DEPCMD_HIPRI_FORCERM)) {
		uint32_t cmdreg = 0U;

		LOG_INF("EP%02x halted with a transfer still pending; recovering "
			"through End Transfer before Clear Stall", ep_cfg->addr);

		/*
		 * 4.2.7 orders End Transfer, then Clear Stall. Issuing them back
		 * to back does not achieve that: udc_dwc3_depcmd()'s pre-poll
		 * finds the End Transfer still active and refuses to write the
		 * Clear Stall at all, so ClearFeature(ENDPOINT_HALT) fails with
		 * -EIO on exactly the endpoints that were busy when they halted,
		 * and stat.halted is left true.
		 */
		if (!udc_dwc3_wait_cmdact_zero(dev,
					       UDC_DWC3_DEPCMD(ep_data->epn),
					       &cmdreg)) {
			LOG_WRN("EP%02x End Transfer still executing (0x%08x); "
				"Clear Stall would be refused, reporting busy",
				ep_cfg->addr, cmdreg);
			return -EBUSY;
		}
	}

	if (!udc_dwc3_depcmd_clear_stall(dev, ep_data,
					 UDC_DWC3_DEPCMD_HIPRI_FORCERM)) {
		/*
		 * Same contract as Set Stall: the flag follows the hardware, not
		 * the intention.
		 */
		LOG_ERR("EP%02x Clear Stall was refused by the controller; endpoint "
			"remains halted", ep_data->cfg.addr);
		return -EIO;
	}

	ep_data->cfg.stat.halted = false;

	/* Resume halted previously transfers */
	k_work_submit_to_queue(udc_get_work_q(), &ep_data->work);

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

/*
 * UDC API: report the negotiated speed.
 */
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

/*
 * UDC API: attach to the bus (DCTL.RunStop).
 */
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

	/* U1/U2 OFF, EXPLICITLY, AND NOT BY LUCK. */
	sys_clear_bits(base + UDC_DWC3_DCTL,
		       UDC_DWC3_DCTL_ACCEPTU1ENA | UDC_DWC3_DCTL_INITU1ENA |
		       UDC_DWC3_DCTL_ACCEPTU2ENA | UDC_DWC3_DCTL_INITU2ENA);

	/* First packet to be expected */

	/* Enable the DWC3 events */
	sys_set_bits(base + UDC_DWC3_DCTL, UDC_DWC3_DCTL_RUNSTOP);

	/* Enable the IRQ (for now, just schedule a first work queue job) */
	cfg->irq_enable_func();

	/* Stamp the control clocks before the housekeeper can ever read them. */
	priv->ctrl_arm_t0 = k_cycle_get_32();
	priv->ctrl_quiet_t0 = priv->ctrl_arm_t0;

	k_timer_start(&priv->heartbeat_timer, K_MSEC(UDC_DWC3_HEARTBEAT_MS),
		      K_MSEC(UDC_DWC3_HEARTBEAT_MS));

	return 0;
}

/*
 * UDC API: detach from the bus.
 */
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
	 * Complete events, so anything outstanding is stranded.
	 */
	udc_dwc3_drop_xfer_state(dev, "controller disable");

	cfg->irq_disable_func();

	return 0;
}

/*
 * UDC API: bring the controller up and enable the control endpoints.
 */
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

/*
 * Endpoint work item: arm queued buffers. Takes the UDC mutex for its whole body.
 */
static void udc_dwc3_ep_worker(struct k_work *const work)
{
	struct udc_dwc3_ep_data *const ep_data = CONTAINER_OF(work, struct udc_dwc3_ep_data, work);
	const struct device *const dev = ep_data->dev;
	struct net_buf *buf;
	int ret;

	LOG_DBG("checking for pending transfers for EP%02x", ep_data->cfg.addr);

	/*
	 * This worker is the one producer of TRBs that did not hold the UDC mutex.
	 */
	udc_lock_internal(dev, K_FOREVER);

	/*
	 * NOT WHILE THE ENDPOINT IS DOWN. This worker can still be queued when
	 * udc_dwc3_ep_disable() clears DALEPENA and releases the ring; arming a
	 * TRB afterwards hands a descriptor to a controller that is no longer
	 * reading the endpoint, and Update Transfer has no resource index to
	 * name. udc_common.c writes stat.enabled under the same mutex this
	 * worker holds, so the read is race-free.
	 */
	if (!ep_data->cfg.stat.enabled || ep_data->cfg.stat.halted) {
		LOG_DBG("endpoint is down or halted, not processing buffers");
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
	 */
	if (udc_dwc3_ep_is_ending(ep_data)) {
		LOG_DBG("EP%02x still concluding an End Transfer, deferring %s",
			ep_data->cfg.addr,
			ep_data->xfer_state == UDC_DWC3_EP_ENDING_RESUME
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
	 * The event ring is drained by its own thread, not by the UDC work queue
	 * - see udc_dwc3_event_thread().
	 */
	/* F12: the vector is connected once, here, not on every enable. */
	if (cfg->irq_connect_func != NULL) {
		cfg->irq_connect_func();
	}

	k_sem_init(&priv->evt_sem, 0, 1);
	priv->evt_stack_free = UDC_DWC3_EVT_STACK_SIZE;

	k_thread_create(priv->evt_thread, priv->evt_stack,
			UDC_DWC3_EVT_STACK_SIZE,
			udc_dwc3_event_thread, priv, NULL, NULL,
			UDC_DWC3_EVT_THREAD_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(priv->evt_thread, "udc_dwc3_evt");

	k_work_init_delayable(&priv->watchdog_dwork, udc_dwc3_watchdog_worker);
	k_work_init(&priv->heartbeat_work, udc_dwc3_heartbeat_worker);
	k_work_init(&priv->nudge_work, udc_dwc3_nudge_worker);
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
	 * The same two initialisations EP0-OUT gets below, and for the same
	 * reasons - EP0-IN was missing both because the pre-init loops start at
	 * i = 1 and the control pair is set up by hand.
	 */
	ep_data->xferrscidx = UDC_DWC3_XFERRSCIDX_INVALID;
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
	ep_data->epn = 0;

	/*
	 * EP0 needs this too. The pre-init loops below start at i = 1, so the
	 * control endpoint never got a k_fifo_init() - yet udc_dwc3_ep_resume()
	 * guards only its first three steps with USB_EP_GET_IDX() > 0 and then
	 * falls through to k_fifo_get() on this queue for every endpoint.
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
		 * NOT ADVERTISED. Every non-control arm goes through
		 * udc_dwc3_trb_bulk(), which writes TRBCTL_NORMAL and leaves the
		 * SOF/uframe field zero, so the controller is given no interval to
		 * start on. Claiming the capability lets a class configure an
		 * endpoint this driver would then mis-arm silently; refusing it at
		 * descriptor-build time fails loudly and early instead.
		 */
		ep_data->cfg.caps.iso = false;
		ep_data->cfg.caps.mps = mps;
		ep_data->trb_buf = cfg->trb_buf_in[i];
		/* Static storage starts at 0, which is a legal index. */
		ep_data->xferrscidx = UDC_DWC3_XFERRSCIDX_INVALID;
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
		/*
		 * NOT ADVERTISED. Every non-control arm goes through
		 * udc_dwc3_trb_bulk(), which writes TRBCTL_NORMAL and leaves the
		 * SOF/uframe field zero, so the controller is given no interval to
		 * start on. Claiming the capability lets a class configure an
		 * endpoint this driver would then mis-arm silently; refusing it at
		 * descriptor-build time fails loudly and early instead.
		 */
		ep_data->cfg.caps.iso = false;
		ep_data->cfg.caps.mps = mps;
		ep_data->trb_buf = cfg->trb_buf_out[i];
		/* Static storage starts at 0, which is a legal index. */
		ep_data->xferrscidx = UDC_DWC3_XFERRSCIDX_INVALID;
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
 */
#define UDC_DWC3_DEVICE_DEFINE(n)						\
	UDC_DWC3_QUIRK_DEFINE(n);						\
										\
	/*								\
	 * F12: IRQ_CONNECT once, from preinit, not on every enable.	\
	 * enable/disable are then only irq_enable()/irq_disable().	\
	 */								\
	static void udc_dwc3_irq_connect_func_##n(void)				\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),		\
			    udc_dwc3_irq_handler, DEVICE_DT_INST_GET(n), 0);	\
	}									\
										\
	static void udc_dwc3_irq_enable_func_##n(void)				\
	{									\
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
		.irq_connect_func = udc_dwc3_irq_connect_func_##n,		\
		.irq_enable_func = udc_dwc3_irq_enable_func_##n,		\
		.irq_disable_func = udc_dwc3_irq_disable_func_##n,		\
	};									\
										\
	K_THREAD_STACK_DEFINE(udc_dwc3_evt_stack_##n,				\
			      UDC_DWC3_EVT_STACK_SIZE);				\
										\
	static struct k_thread udc_dwc3_evt_thread_##n;				\
										\
	static struct udc_dwc3_data udc_dwc3_priv_##n = {			\
		.dev = DEVICE_DT_INST_GET(n),					\
		.evt_stack = udc_dwc3_evt_stack_##n,				\
		.evt_thread = &udc_dwc3_evt_thread_##n,				\
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
		 * than the queues being absent.
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
			priv->max_bytes_avail[n][i] = udc_dwc3_read_fifo_space(
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
		char *s = (i == priv->evt_next) ? "<-" : "  ";

		shell_print(sh, "evt 0x%02x: 0x%08x %s %s",
			i, evt, s, udc_dwc3_get_event_name(evt, 0));
	}

	/*
	 * How often the controller's posted write had not landed when the event
	 * was read.
	 */
	shell_print(sh, "events %u, posted-write waits: late %u, gave up %u",
		    priv->evt_handled, priv->evt_late, priv->evt_gaveup);
	shell_print(sh, "worst late wait: %u polls (lower bound), %u us (upper bound)",
		    priv->evt_late_polls_max, priv->evt_late_us_max);
	shell_print(sh, "drain re-armed after unmask: %u", priv->evt_rearm);
	shell_print(sh, "link state changes %u, last state 0x%x, repeated x%u",
		    priv->evt_link_total, priv->evt_link_last, priv->evt_link_run);
	shell_print(sh, "GEVNTCOUNT high-water %u bytes of %u, drain %s, "
		    "give-up run %u on slot %u",
		    priv->evt_gevntcount_hwm,
		    (unsigned int)(CONFIG_UDC_DWC3_EVENTS_NUM * sizeof(uint32_t)),
		    udc_dwc3_drain_state_name(priv->drain.state),
		    priv->drain.attempts, priv->drain.slot);
	shell_print(sh, "control aborts: setup-pending %u, other TRBSTS %u, "
		    "stage desync %u", priv->ctrl_setup_pending,
		    priv->ctrl_trbsts_other, priv->ctrl_desync);
	shell_print(sh, "control arms deferred behind an End Transfer: %u",
		    priv->ctrl_deferred_arm);
}

/*
 * Log one TRB's fields.
 */
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
				(uint32_t)priv->max_bytes_avail[n][i]);
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

/* Defined below; compared against to spot the one self-locking callback. */
static void udc_dwc3_cmd_recover(const struct device *dev, const struct shell *sh);

/*
 * Every dwc3 shell command comes through here, and every one of them touches
 * driver state that the work queue and the ISR are also touching: arming control
 * TRBs, issuing endpoint commands, walking the TRB rings and the event buffer.
 * None of it was serialised against the driver, so a command typed while traffic
 * was running raced the event worker - and the shell exists precisely to be used
 * while something is going wrong, which is the worst moment to corrupt state.
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

/*
 * Shell: arm an EP0-IN status TRB by hand.
 */
static void udc_dwc3_cmd_trb_ctrl_status_in(const struct device *dev, const struct shell *sh)
{
	struct net_buf *buf;

	shell_print(sh, "New UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3 IN");

	buf = udc_ep_buf_alloc(dev, USB_CONTROL_EP_IN, 128);
	if (buf == NULL) {
		shell_error(sh, "Failed to allocate a buffer");
		return;
	}

	(void)udc_dwc3_trb_ctrl_in(dev, buf, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3);
}
static int cmd_dwc3_trb_ctrl_status_in(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_cmd_trb_ctrl_status_in);
}

/*
 * Shell: arm an EP0-OUT status TRB by hand.
 */
static void udc_dwc3_cmd_trb_ctrl_status_out(const struct device *dev, const struct shell *sh)
{
	struct net_buf *buf;

	shell_print(sh, "New UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3 OUT");

	buf = udc_ep_buf_alloc(dev, USB_CONTROL_EP_OUT, 128);
	if (buf == NULL) {
		shell_error(sh, "Failed to allocate a buffer");
		return;
	}

	(void)udc_dwc3_trb_ctrl_out(dev, buf, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3);
}
static int cmd_dwc3_trb_ctrl_status_out(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_cmd_trb_ctrl_status_out);
}

/*
 * Shell: arm an EP0-OUT data TRB by hand.
 */
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

	(void)udc_dwc3_trb_ctrl_out(dev, buf, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA);
}
static int cmd_dwc3_trb_ctrl_data_out(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_cmd_trb_ctrl_data_out);
}

/*
 * Shell: arm an EP0-IN data TRB by hand.
 */
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

	(void)udc_dwc3_trb_ctrl_in(dev, buf, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA);
}
static int cmd_dwc3_trb_ctrl_data_in(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_cmd_trb_ctrl_data_in);
}

/*
 * Shell: arm an EP0-OUT SETUP TRB by hand.
 */
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

	(void)udc_dwc3_trb_ctrl_out(dev, buf, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_SETUP);
}
static int cmd_dwc3_trb_ctrl_setup(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_cmd_trb_ctrl_setup);
}

/*
 * End Transfer on a LIVE control endpoint hangs this controller - a capture
 * caught DEPCMD 0x00000d08 on EP0 with CmdAct still set after 1000 ms, sixty
 * times over, while DSTS showed the link healthy in U0. The driver refuses it
 * everywhere else for that reason; these two shell commands were the one way
 * left to do it by hand. Still allowed when the endpoint is genuinely idle,
 * which is when it is useful for diagnosis.
 */
static bool udc_dwc3_shell_ctrl_end_ok(const struct udc_dwc3_ep_data *const ep_data,
				       const struct shell *sh)
{
	const volatile struct udc_dwc3_trb *const trb = ep_data->trb_buf;

	if ((trb[0].ctrl & UDC_DWC3_TRB_CTRL_HWO) != 0U) {
		shell_error(sh, "EP%02x still owns a TRB (ctrl 0x%08x): End Transfer "
			    "on a live control endpoint hangs this controller. Refused.",
			    ep_data->cfg.addr, trb[0].ctrl);
		return false;
	}

	if (udc_ep_is_busy(&ep_data->cfg)) {
		shell_error(sh, "EP%02x is claimed by a transfer in progress. Refused.",
			    ep_data->cfg.addr);
		return false;
	}

	return true;
}

/*
 * Shell: End Transfer on EP0-IN.
 */
static void udc_dwc3_cmd_end_ctrl_in(const struct device *dev, const struct shell *sh)
{
	const struct udc_dwc3_config *const cfg = dev->config;

	if (udc_dwc3_shell_ctrl_end_ok(&cfg->ep_data_in[0], sh)) {
		udc_dwc3_depcmd_end_xfer(dev, &cfg->ep_data_in[0], 0);
	}
}
static int cmd_dwc3_end_ctrl_in(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_cmd_end_ctrl_in);
}

/*
 * Shell: End Transfer on EP0-OUT.
 */
static void udc_dwc3_cmd_end_ctrl_out(const struct device *dev, const struct shell *sh)
{
	const struct udc_dwc3_config *const cfg = dev->config;

	if (udc_dwc3_shell_ctrl_end_ok(&cfg->ep_data_out[0], sh)) {
		udc_dwc3_depcmd_end_xfer(dev, &cfg->ep_data_out[0],
					 UDC_DWC3_DEPCMD_HIPRI_FORCERM);
	}
}
static int cmd_dwc3_end_ctrl_out(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_cmd_end_ctrl_out);
}

/*
 * Shell: inject a synthetic XferComplete.
 */
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
 * Shell: Set Stall on EP0-OUT.
 */
static void udc_dwc3_cmd_dwc3_stall_ctrl_out(const struct device *const dev, const struct shell *sh)
{
	const struct udc_dwc3_config *const cfg = dev->config;

	udc_dwc3_depcmd_set_stall(dev, &cfg->ep_data_out[0]);
}
static int cmd_dwc3_stall_ctrl_out(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_cmd_dwc3_stall_ctrl_out);
}

/*
 * Shell: Set Stall on EP0-IN.
 */
static void udc_dwc3_cmd_dwc3_stall_ctrl_in(const struct device *const dev, const struct shell *sh)
{
	const struct udc_dwc3_config *const cfg = dev->config;

	udc_dwc3_depcmd_set_stall(dev, &cfg->ep_data_in[0]);
}
static int cmd_dwc3_stall_ctrl_in(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_cmd_dwc3_stall_ctrl_in);
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