/*
 * SPDX-FileCopyrightText: Copyright tinyVision.ai Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Verified transfer-arm for CPU-managed DWC3 endpoints.
 *
 * DEPCMD completion is telemetry only. Success means the hardware is
 * actually working the ring (tail TRB HWO cleared). Call sites pick a
 * named policy; they do not open-code settle loops.
 */

#ifndef ZEPHYR_DRIVERS_USB_UDC_DWC3_ARM_H_
#define ZEPHYR_DRIVERS_USB_UDC_DWC3_ARM_H_

#include <stdint.h>
#include <stdbool.h>

struct device;
struct udc_dwc3_ep_data;
struct udc_dwc3_in_recovery_stats;

/** Which doorbell / command sequence to run before verify. */
enum udc_dwc3_arm_op {
	/** DepStartXfer, then verify (UpdateXfer / optional Restart). */
	UDC_DWC3_ARM_START = 0,
	/** DepUpdateXfer only, then verify (nudge an existing resource). */
	UDC_DWC3_ARM_UPDATE,
	/** EndXfer(ForceRM)+StartXfer, then verify. */
	UDC_DWC3_ARM_RESTART,
};

/**
 * Outcome of udc_dwc3_arm_transfer().
 * OK_* means HWO cleared. STUCK means still armed-but-dead after policy.
 */
enum udc_dwc3_arm_result {
	UDC_DWC3_ARM_OK_FAST = 0,
	UDC_DWC3_ARM_OK_NUDGED,
	UDC_DWC3_ARM_OK_RESTARTED,
	UDC_DWC3_ARM_BACKOFF,
	UDC_DWC3_ARM_STUCK,
};

/**
 * Timing / escalate knobs for one arm attempt.
 * Prefer the named presets over ad-hoc values at call sites.
 */
struct udc_dwc3_arm_policy {
	uint32_t settle_us;
	uint8_t fast_steps;
	uint8_t update_retries;
	uint8_t update_settle_steps;
	uint8_t restart_settle_steps;
	/** If true, START/UPDATE may EndXfer+StartXfer once before STUCK. */
	bool allow_restart;
	const char *name;
};

/** Steady-state CPU IN arm (ACM / CDC-RAW under normal load). */
const struct udc_dwc3_arm_policy *udc_dwc3_arm_policy_in_normal(void);

/**
 * First arm after a pipe rebuild (ring nuke + requeue).
 * Longer settle: shared DEPCMD path is still hot from EndXfer storm.
 */
const struct udc_dwc3_arm_policy *udc_dwc3_arm_policy_in_after_rebuild(void);

/**
 * Issue @p op and verify until HWO clears or the policy is exhausted.
 *
 * @retval UDC_DWC3_ARM_OK_*     HW fetched the tail TRB
 * @retval UDC_DWC3_ARM_BACKOFF  UpdateXfer CMDERR; try again later
 * @retval UDC_DWC3_ARM_STUCK    still HWO=1 after full ladder
 */
enum udc_dwc3_arm_result udc_dwc3_arm_transfer(const struct device *dev,
					       struct udc_dwc3_ep_data *ep_data,
					       enum udc_dwc3_arm_op op,
					       const struct udc_dwc3_arm_policy *policy);

/**
 * Host-safe give-up: EndXfer + SetStall + clear SW arm bookkeeping.
 * Prefer this over leaving a ghost HWO=1 IN that wedges host xHCI.
 */
void udc_dwc3_arm_give_up(const struct device *dev,
			  struct udc_dwc3_ep_data *ep_data,
			  const char *reason);

/** Fill arm_* fields of the soak stats struct. */
void udc_dwc3_arm_stats_get(struct udc_dwc3_in_recovery_stats *stats);

/** True if @p result means the pipe is live. */
static inline bool udc_dwc3_arm_ok(enum udc_dwc3_arm_result result)
{
	return result == UDC_DWC3_ARM_OK_FAST ||
	       result == UDC_DWC3_ARM_OK_NUDGED ||
	       result == UDC_DWC3_ARM_OK_RESTARTED;
}

#endif /* ZEPHYR_DRIVERS_USB_UDC_DWC3_ARM_H_ */
