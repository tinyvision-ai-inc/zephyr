/*
 * SPDX-FileCopyrightText: Copyright tinyVision.ai Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Verified transfer-arm for CPU-managed DWC3 endpoints.
 *
 * DEPCMD completion is telemetry only. Call sites pick a named policy that
 * defines what "armed" means:
 *   - IN / OUT run-dry: tail HWO cleared (HW worked the TRB)
 *   - OUT park (StartXfer): resource live with HWO=1 is success (idle RX)
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
	/** DepStartXfer, then verify per policy. */
	UDC_DWC3_ARM_START = 0,
	/** DepUpdateXfer only, then verify (nudge an existing resource). */
	UDC_DWC3_ARM_UPDATE,
	/** EndXfer(ForceRM)+StartXfer, then verify. */
	UDC_DWC3_ARM_RESTART,
};

/**
 * What counts as success after the command ladder.
 * OUT park must not use HWO_CLEAR — idle OUT keeps HWO=1 until the host writes.
 */
enum udc_dwc3_arm_success {
	/** Tail HWO cleared (IN steady-state / OUT run-dry). */
	UDC_DWC3_ARM_EXPECT_HWO_CLEAR = 0,
	/** Start/Update issued; HWO=1 still means parked-and-live (OUT Start). */
	UDC_DWC3_ARM_EXPECT_ARMED,
};

/**
 * Outcome of udc_dwc3_arm_transfer().
 * OK_FAST/NUDGED/RESTARTED: HWO cleared.
 * OK_ARMED: OUT park — command took, waiting for host.
 * STUCK: policy exhausted without success.
 */
enum udc_dwc3_arm_result {
	UDC_DWC3_ARM_OK_FAST = 0,
	UDC_DWC3_ARM_OK_NUDGED,
	UDC_DWC3_ARM_OK_RESTARTED,
	UDC_DWC3_ARM_OK_ARMED,
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
	/**
	 * When allow_restart: use OUT EndXfer recycle (leaves ring pointers)
	 * instead of the IN EndXfer+StartHWO poll helper.
	 */
	bool use_out_recycle;
	enum udc_dwc3_arm_success success;
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
 * OUT StartXfer / park: HWO=1 after a successful DepStartXfer is success.
 * Brief HWO-clear poll still allowed (host may have already written).
 */
const struct udc_dwc3_arm_policy *udc_dwc3_arm_policy_out_park(void);

/**
 * OUT run-dry recovery: expect HWO clear under host write pressure;
 * EndXfer+StartXfer recycle once if UpdateXfer ladder fails.
 */
const struct udc_dwc3_arm_policy *udc_dwc3_arm_policy_out_rundry(void);

/**
 * Issue @p op and verify per @p policy.
 *
 * @retval UDC_DWC3_ARM_OK_*     success per policy->success
 * @retval UDC_DWC3_ARM_BACKOFF  UpdateXfer CMDERR; try again later
 * @retval UDC_DWC3_ARM_STUCK    policy exhausted
 */
enum udc_dwc3_arm_result udc_dwc3_arm_transfer(const struct device *dev,
					       struct udc_dwc3_ep_data *ep_data,
					       enum udc_dwc3_arm_op op,
					       const struct udc_dwc3_arm_policy *policy);

/**
 * Terminal give-up: EndXfer + SetStall. Avoid during set_configuration —
 * stalling ACM/CDC IN there prevents enumeration.
 * Prefer this over leaving a ghost HWO=1 IN that wedges host xHCI.
 */
void udc_dwc3_arm_give_up(const struct device *dev,
			  struct udc_dwc3_ep_data *ep_data,
			  const char *reason);

/** Fill arm_* fields of the soak stats struct. */
void udc_dwc3_arm_stats_get(struct udc_dwc3_in_recovery_stats *stats);

/** True if @p result means the pipe is live (including parked OUT). */
static inline bool udc_dwc3_arm_ok(enum udc_dwc3_arm_result result)
{
	return result == UDC_DWC3_ARM_OK_FAST ||
	       result == UDC_DWC3_ARM_OK_NUDGED ||
	       result == UDC_DWC3_ARM_OK_RESTARTED ||
	       result == UDC_DWC3_ARM_OK_ARMED;
}

#endif /* ZEPHYR_DRIVERS_USB_UDC_DWC3_ARM_H_ */
