/*
 * SPDX-FileCopyrightText: Copyright tinyVision.ai Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Verified arm: issue a DWC3 transfer command, then resolve HWO / park
 * unknowns according to a named policy (IN vs OUT).
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/usb/usb_ch9.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(dwc3, CONFIG_UDC_DRIVER_LOG_LEVEL);

#include "udc_dwc3_int.h"
#include "udc_dwc3_arm.h"

static const struct udc_dwc3_arm_policy policy_in_normal = {
	.settle_us = 10U,
	.fast_steps = 8U,
	.update_retries = 6U,
	.update_settle_steps = 8U,
	.restart_settle_steps = 16U,
	.allow_restart = false,
	.use_out_recycle = false,
	.success = UDC_DWC3_ARM_EXPECT_HWO_CLEAR,
	.name = "in_normal",
};

static const struct udc_dwc3_arm_policy policy_in_after_rebuild = {
	.settle_us = 50U,
	.fast_steps = 40U,
	.update_retries = 12U,
	.update_settle_steps = 40U,
	.restart_settle_steps = 64U,
	.allow_restart = true,
	.use_out_recycle = false,
	.success = UDC_DWC3_ARM_EXPECT_HWO_CLEAR,
	.name = "in_after_rebuild",
};

/* OUT StartXfer: parked RX with HWO=1 is the steady state. */
static const struct udc_dwc3_arm_policy policy_out_park = {
	.settle_us = 10U,
	.fast_steps = 2U,
	.update_retries = 0U,
	.update_settle_steps = 0U,
	.restart_settle_steps = 0U,
	.allow_restart = false,
	.use_out_recycle = false,
	.success = UDC_DWC3_ARM_EXPECT_ARMED,
	.name = "out_park",
};

/*
 * OUT run-dry: host should be writing / completion pending — require HWO clear.
 * Matches former sm_out_update_verify settle budget; recycle via OUT EndXfer.
 */
static const struct udc_dwc3_arm_policy policy_out_rundry = {
	.settle_us = 50U,
	.fast_steps = 0U,
	.update_retries = 8U,
	.update_settle_steps = 16U,
	.restart_settle_steps = 16U,
	.allow_restart = true,
	.use_out_recycle = true,
	.success = UDC_DWC3_ARM_EXPECT_HWO_CLEAR,
	.name = "out_rundry",
};

static atomic_t arm_ok_fast;
static atomic_t arm_ok_nudge;
static atomic_t arm_nudge_sum;
static atomic_t arm_nudge_max;
static atomic_t arm_fail;
static atomic_t arm_backoff;
static atomic_t arm_give_up;

const struct udc_dwc3_arm_policy *udc_dwc3_arm_policy_in_normal(void)
{
	return &policy_in_normal;
}

const struct udc_dwc3_arm_policy *udc_dwc3_arm_policy_in_after_rebuild(void)
{
	return &policy_in_after_rebuild;
}

const struct udc_dwc3_arm_policy *udc_dwc3_arm_policy_out_park(void)
{
	return &policy_out_park;
}

const struct udc_dwc3_arm_policy *udc_dwc3_arm_policy_out_rundry(void)
{
	return &policy_out_rundry;
}

void udc_dwc3_arm_stats_get(struct udc_dwc3_in_recovery_stats *const stats)
{
	if (stats == NULL) {
		return;
	}

	stats->arm_ok_fast = atomic_get(&arm_ok_fast);
	stats->arm_ok_nudge = atomic_get(&arm_ok_nudge);
	stats->arm_nudge_sum = atomic_get(&arm_nudge_sum);
	stats->arm_nudge_max = atomic_get(&arm_nudge_max);
	stats->arm_fail = atomic_get(&arm_fail);
}

static const char *arm_dir_tag(const struct udc_dwc3_ep_data *ep_data)
{
	return USB_EP_DIR_IS_IN(ep_data->cfg.addr) ? "IN-ARM" : "OUT-ARM";
}

static void arm_note_nudge_max(unsigned int nudges)
{
	atomic_val_t cur;

	do {
		cur = atomic_get(&arm_nudge_max);
		if ((atomic_val_t)nudges <= cur) {
			return;
		}
	} while (!atomic_cas(&arm_nudge_max, cur, (atomic_val_t)nudges));
}

static void arm_record_ok(struct udc_dwc3_ep_data *ep_data, unsigned int nudges,
			  enum udc_dwc3_arm_result result, const char *policy_name)
{
	const char *how;

	switch (result) {
	case UDC_DWC3_ARM_OK_FAST:
		how = "fast";
		break;
	case UDC_DWC3_ARM_OK_NUDGED:
		how = "nudge";
		break;
	case UDC_DWC3_ARM_OK_RESTARTED:
		how = "restart";
		break;
	case UDC_DWC3_ARM_OK_ARMED:
		how = "armed";
		break;
	default:
		how = "?";
		break;
	}

	if (result == UDC_DWC3_ARM_OK_ARMED) {
		atomic_inc(&arm_ok_fast);
		LOG_DBG("%s: ep=0x%02x OK how=%s policy=%s (parked HWO=1)",
			arm_dir_tag(ep_data), ep_data->cfg.addr, how, policy_name);
	} else if (nudges == 0U) {
		atomic_inc(&arm_ok_fast);
		LOG_DBG("%s: ep=0x%02x OK how=%s policy=%s",
			arm_dir_tag(ep_data), ep_data->cfg.addr, how, policy_name);
	} else {
		atomic_inc(&arm_ok_nudge);
		atomic_add(&arm_nudge_sum, (atomic_val_t)nudges);
		arm_note_nudge_max(nudges);
		/* Per-transfer nudge OK is normal under bulk OUT; use DBG.
		 * Soak rollups come from IN-ARM-STATS (ok_nudge / nudge_max).
		 */
		LOG_DBG("%s: ep=0x%02x OK how=%s policy=%s update_db=%u (HWO cleared)",
			arm_dir_tag(ep_data), ep_data->cfg.addr, how, policy_name, nudges);
	}

#if defined(CONFIG_UDC_DWC3_IN_START_ENDXFER_ESCALATE)
	if (USB_EP_DIR_IS_IN(ep_data->cfg.addr)) {
		/*
		 * A verified live IN pipe clears the rebuild budget so one bad
		 * stretch does not permanently force drop-mode mid-transfer.
		 */
		ep_data->rebuild_attempts = 0U;
	}
#endif
#if defined(CONFIG_UDC_DWC3_EP_SM)
	ep_data->sm.arm_fail_logged = false;
	ep_data->sm.arm_after_rebuild = false;
#endif
}

static bool arm_tail_hwo_cleared(const struct udc_dwc3_ep_data *ep_data)
{
	return !udc_dwc3_int_trb_hwo(&ep_data->trb_buf[ep_data->tail]);
}

static bool arm_poll_hwo_clear(struct udc_dwc3_ep_data *ep_data,
			       unsigned int steps, uint32_t settle_us)
{
	for (unsigned int step = 0U; step < steps; step++) {
		if (arm_tail_hwo_cleared(ep_data)) {
			return true;
		}
		k_busy_wait(settle_us);
	}

	return arm_tail_hwo_cleared(ep_data);
}

static void arm_issue(const struct device *dev, struct udc_dwc3_ep_data *ep_data,
		      enum udc_dwc3_arm_op op)
{
	switch (op) {
	case UDC_DWC3_ARM_START:
		udc_dwc3_int_depcmd_start_xfer(dev, ep_data);
		break;
	case UDC_DWC3_ARM_UPDATE:
		udc_dwc3_int_depcmd_update_xfer(dev, ep_data);
		break;
	case UDC_DWC3_ARM_RESTART:
	default:
		break;
	}
}

static bool arm_try_restart(const struct device *dev,
			    struct udc_dwc3_ep_data *ep_data,
			    const struct udc_dwc3_arm_policy *policy)
{
	if (policy->use_out_recycle) {
		return udc_dwc3_int_out_endxfer_recycle(dev, ep_data);
	}

	return udc_dwc3_int_in_endxfer_retry(dev, ep_data,
					     policy->restart_settle_steps,
					     policy->settle_us);
}

enum udc_dwc3_arm_result udc_dwc3_arm_transfer(const struct device *dev,
					       struct udc_dwc3_ep_data *ep_data,
					       enum udc_dwc3_arm_op op,
					       const struct udc_dwc3_arm_policy *policy)
{
	unsigned int update_db = 0U;
	const char *pname;
	const char *tag;

	if (dev == NULL || ep_data == NULL || policy == NULL) {
		return UDC_DWC3_ARM_STUCK;
	}

	pname = policy->name != NULL ? policy->name : "?";
	tag = arm_dir_tag(ep_data);

#if defined(CONFIG_UDC_DWC3_EP_SM)
	ep_data->sm.arm_verify_busy = true;
#endif

	if (op == UDC_DWC3_ARM_RESTART) {
		if (!arm_try_restart(dev, ep_data, policy)) {
#if defined(CONFIG_UDC_DWC3_EP_SM)
			ep_data->sm.arm_verify_busy = false;
#endif
			atomic_inc(&arm_fail);
			return UDC_DWC3_ARM_STUCK;
		}
		arm_record_ok(ep_data, 0U, UDC_DWC3_ARM_OK_RESTARTED, pname);
#if defined(CONFIG_UDC_DWC3_EP_SM)
		ep_data->sm.arm_verify_busy = false;
#endif
		return UDC_DWC3_ARM_OK_RESTARTED;
	}

	arm_issue(dev, ep_data, op);

	/* Optional fast HWO-clear window (host already completed). */
	if (policy->fast_steps > 0U &&
	    arm_poll_hwo_clear(ep_data, policy->fast_steps, policy->settle_us)) {
		arm_record_ok(ep_data, 0U, UDC_DWC3_ARM_OK_FAST, pname);
#if defined(CONFIG_UDC_DWC3_EP_SM)
		ep_data->sm.arm_verify_busy = false;
#endif
		return UDC_DWC3_ARM_OK_FAST;
	}

	/*
	 * OUT park: DepStartXfer issued and HWO still 1 → armed, waiting for
	 * host. Do not UpdateXfer-storm or declare STUCK.
	 */
	if (policy->success == UDC_DWC3_ARM_EXPECT_ARMED) {
		arm_record_ok(ep_data, 0U, UDC_DWC3_ARM_OK_ARMED, pname);
#if defined(CONFIG_UDC_DWC3_EP_SM)
		ep_data->sm.arm_verify_busy = false;
#endif
		return UDC_DWC3_ARM_OK_ARMED;
	}

	/* Nudge path: DepUpdateXfer until HWO clears or CMDERR. */
	for (unsigned int attempt = 0U; attempt < policy->update_retries; attempt++) {
		bool cmderr = false;

		(void)udc_dwc3_int_depcmd_update_xfer_checked(dev, ep_data, &cmderr);
		update_db++;

		if (arm_poll_hwo_clear(ep_data, policy->update_settle_steps,
				       policy->settle_us)) {
			arm_record_ok(ep_data, update_db, UDC_DWC3_ARM_OK_NUDGED, pname);
#if defined(CONFIG_UDC_DWC3_EP_SM)
			ep_data->sm.arm_verify_busy = false;
#endif
			return UDC_DWC3_ARM_OK_NUDGED;
		}

		if (cmderr) {
			atomic_inc(&arm_backoff);
			LOG_WRN("%s: ep=0x%02x BACKOFF policy=%s update_db=%u CMDERR",
				tag, ep_data->cfg.addr, pname, update_db);
#if defined(CONFIG_UDC_DWC3_EP_SM)
			ep_data->sm.arm_verify_busy = false;
#endif
			return UDC_DWC3_ARM_BACKOFF;
		}
	}

	/* Optional EndXfer+StartXfer once (IN rebuild / OUT run-dry). */
	if (policy->allow_restart) {
		LOG_WRN("%s: ep=0x%02x RESTART policy=%s after update_db=%u still hwo=1",
			tag, ep_data->cfg.addr, pname, update_db);
		if (arm_try_restart(dev, ep_data, policy)) {
			arm_record_ok(ep_data, update_db, UDC_DWC3_ARM_OK_RESTARTED, pname);
#if defined(CONFIG_UDC_DWC3_EP_SM)
			ep_data->sm.arm_verify_busy = false;
#endif
			return UDC_DWC3_ARM_OK_RESTARTED;
		}
	}

	atomic_inc(&arm_fail);
#if defined(CONFIG_UDC_DWC3_EP_SM)
	if (!ep_data->sm.arm_fail_logged) {
		ep_data->sm.arm_fail_logged = true;
		LOG_ERR("%s: ep=0x%02x STUCK policy=%s update_db=%u hwo=1",
			tag, ep_data->cfg.addr, pname, update_db);
	}
	ep_data->sm.arm_verify_busy = false;
	ep_data->sm.arm_after_rebuild = false;
#else
	LOG_ERR("%s: ep=0x%02x STUCK policy=%s update_db=%u hwo=1",
		tag, ep_data->cfg.addr, pname, update_db);
#endif

	return UDC_DWC3_ARM_STUCK;
}

void udc_dwc3_arm_give_up(const struct device *dev,
			  struct udc_dwc3_ep_data *ep_data,
			  const char *reason)
{
	if (dev == NULL || ep_data == NULL) {
		return;
	}

	atomic_inc(&arm_give_up);
	LOG_ERR("%s: ep=0x%02x GIVE-UP reason=%s — EndXfer+SetStall",
		arm_dir_tag(ep_data), ep_data->cfg.addr,
		reason != NULL ? reason : "?");

	/* Prefer not to call this during configure; see pipe-rebuild drop path. */
	udc_dwc3_int_cpu_ep_halt(dev, ep_data);

#if defined(CONFIG_UDC_DWC3_EP_SM)
	ep_data->sm.arm_verify_busy = false;
	ep_data->sm.arm_after_rebuild = false;
	ep_data->sm.rebuild_in_progress = false;
	udc_dwc3_ep_sm_set_state(ep_data, UDC_DWC3_EP_SM_HALTED);
#endif
	ep_data->xfer_active = false;
}
