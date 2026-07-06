/*
 * SPDX-FileCopyrightText: Copyright tinyVision.ai Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Single state machine for CPU-terminated bulk endpoint DWC3 interactions.
 * HWO on the tail TRB is ground truth; DEPEVT and periodic poll are hints.
 */

#define DT_DRV_COMPAT snps_dwc3

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/usb/usb_ch9.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(dwc3, CONFIG_UDC_DRIVER_LOG_LEVEL);

#include "udc_dwc3_int.h"

#define UDC_DWC3_TRB_CTRL_HWO BIT(0)

#define UDC_DWC3_RUNDRY_REARM_RETRIES  8U
#define UDC_DWC3_RUNDRY_SETTLE_STEPS   16U
#define UDC_DWC3_RUNDRY_SETTLE_US      50U

#define UDC_DWC3_INSTART_SETTLE_US     10U
#define UDC_DWC3_INSTART_FAST_STEPS    8U
#define UDC_DWC3_INSTART_REARM_RETRIES 6U
#define UDC_DWC3_INSTART_SETTLE_STEPS  8U
#define UDC_DWC3_INSTART_RECYCLE_STEPS 16U

#define UDC_DWC3_DEPEVT_EPN_MASK       GENMASK(5, 1)

static atomic_t udc_dwc3_sm_in_start_retook;
static atomic_t udc_dwc3_sm_in_start_exhausted;
static atomic_t udc_dwc3_sm_in_start_recycled;
static atomic_t udc_dwc3_sm_poll_recovered;

bool udc_dwc3_ep_sm_is_cpu(const struct udc_dwc3_ep_data *ep_data)
{
	const uint8_t addr = ep_data->cfg.addr;

	if (addr == USB_CONTROL_EP_IN || addr == USB_CONTROL_EP_OUT) {
		return false;
	}

	/* Video stream bulk IN endpoints — not managed by this SM. */
	if (addr == 0x83 || addr == 0x84) {
		return false;
	}

	return ep_data->cfg.caps.bulk;
}

void udc_dwc3_ep_sm_init(struct udc_dwc3_ep_data *ep_data)
{
	ep_data->sm.state = UDC_DWC3_EP_SM_IDLE;
	ep_data->sm.poll_grace_armed = false;
}

void udc_dwc3_ep_sm_set_state(struct udc_dwc3_ep_data *ep_data,
			      enum udc_dwc3_ep_sm_state state)
{
	ep_data->sm.state = state;
}

static const char *udc_dwc3_sm_reason_str(enum udc_dwc3_ep_adv_reason reason)
{
	switch (reason) {
	case UDC_DWC3_EP_ADV_DEPEVT:
		return "depevt";
	case UDC_DWC3_EP_ADV_POLL:
		return "poll";
	case UDC_DWC3_EP_ADV_DOORBELL:
		return "doorbell";
	case UDC_DWC3_EP_ADV_WORKER:
		return "worker";
	default:
		return "?";
	}
}

#if defined(CONFIG_UDC_DWC3_EP_SM_LOG_PHASE)
static const char *udc_dwc3_sm_state_str(enum udc_dwc3_ep_sm_state state)
{
	switch (state) {
	case UDC_DWC3_EP_SM_IDLE:
		return "idle";
	case UDC_DWC3_EP_SM_ACTIVE:
		return "active";
	case UDC_DWC3_EP_SM_HALTED:
		return "halted";
	case UDC_DWC3_EP_SM_CLEAR_PENDING:
		return "clear_pending";
	default:
		return "?";
	}
}

static const char *udc_dwc3_sm_doorbell_str(enum udc_dwc3_doorbell_cmd cmd)
{
	switch (cmd) {
	case UDC_DWC3_DB_START:
		return "start";
	case UDC_DWC3_DB_UPDATE:
		return "update";
	case UDC_DWC3_DB_UPDATE_VERIFY:
		return "update_verify";
	default:
		return "?";
	}
}
#endif


static void udc_dwc3_sm_in_start_verify(const struct device *dev,
					struct udc_dwc3_ep_data *ep_data)
{
	const uint32_t tail = ep_data->tail;

	for (unsigned int step = 0U; step < UDC_DWC3_INSTART_FAST_STEPS; step++) {
		if (!udc_dwc3_int_trb_hwo(&ep_data->trb_buf[tail])) {
			return;
		}
		k_busy_wait(UDC_DWC3_INSTART_SETTLE_US);
	}

	for (unsigned int attempt = 0U; attempt < UDC_DWC3_INSTART_REARM_RETRIES; attempt++) {
		udc_dwc3_int_depcmd_update_xfer(dev, ep_data);

		for (unsigned int step = 0U; step < UDC_DWC3_INSTART_SETTLE_STEPS; step++) {
			k_busy_wait(UDC_DWC3_INSTART_SETTLE_US);
			if (!udc_dwc3_int_trb_hwo(&ep_data->trb_buf[tail])) {
				atomic_inc(&udc_dwc3_sm_in_start_retook);
				return;
			}
		}
	}

	LOG_WRN("EP-SM: IN-START-REARM ep=0x%02x verify exhausted", ep_data->cfg.addr);
#if defined(CONFIG_UDC_DWC3_IN_START_ENDXFER_ESCALATE)
	udc_dwc3_int_in_endxfer_recycle(dev, ep_data);
#else
	atomic_inc(&udc_dwc3_sm_in_start_exhausted);
#endif
}

static void udc_dwc3_sm_out_update_verify(const struct device *dev,
					  struct udc_dwc3_ep_data *ep_data)
{
	const uint32_t tail = ep_data->tail;

	for (unsigned int attempt = 0U; attempt < UDC_DWC3_RUNDRY_REARM_RETRIES; attempt++) {
		udc_dwc3_int_depcmd_update_xfer(dev, ep_data);

		for (unsigned int step = 0U; step < UDC_DWC3_RUNDRY_SETTLE_STEPS; step++) {
			k_busy_wait(UDC_DWC3_RUNDRY_SETTLE_US);
			if (!udc_dwc3_int_trb_hwo(&ep_data->trb_buf[tail])) {
				return;
			}
		}
	}

	LOG_WRN("EP-SM: OUT-RUNDRY ep=0x%02x update verify exhausted tail=%u "
		"ctl=0x%08x sts=0x%08x hwo=%d head=%u active=%d",
		ep_data->cfg.addr, tail, ep_data->trb_buf[tail].ctrl,
		ep_data->trb_buf[tail].status,
		udc_dwc3_int_trb_hwo(&ep_data->trb_buf[tail]) ? 1 : 0,
		ep_data->head, ep_data->xfer_active);
}

int udc_dwc3_doorbell_issue(const struct device *dev,
			    struct udc_dwc3_ep_data *ep_data,
			    enum udc_dwc3_doorbell_cmd cmd)
{
	if (!udc_dwc3_ep_sm_is_cpu(ep_data)) {
		return -ENOTSUP;
	}

#if defined(CONFIG_UDC_DWC3_EP_SM_LOG_PHASE)
	if (!udc_dwc3_int_bulk_eps_live(dev)) {
		LOG_WRN("EP-SM: DOORBELL-PREMATURE ep=0x%02x cmd=%s live=0 state=%s",
			ep_data->cfg.addr, udc_dwc3_sm_doorbell_str(cmd),
			udc_dwc3_sm_state_str(ep_data->sm.state));
	}
#endif

	switch (cmd) {
	case UDC_DWC3_DB_START:
		udc_dwc3_int_depcmd_start_xfer(dev, ep_data);
		ep_data->sm.state = UDC_DWC3_EP_SM_ACTIVE;
		if (USB_EP_DIR_IS_IN(ep_data->cfg.addr)) {
			udc_dwc3_sm_in_start_verify(dev, ep_data);
		}
		break;
	case UDC_DWC3_DB_UPDATE:
		udc_dwc3_int_depcmd_update_xfer(dev, ep_data);
		break;
	case UDC_DWC3_DB_UPDATE_VERIFY:
		udc_dwc3_sm_out_update_verify(dev, ep_data);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static bool udc_dwc3_sm_tail_retire_eligible(const struct udc_dwc3_ep_data *ep_data)
{
	const uint32_t tail = ep_data->tail;

	return ep_data->net_buf[tail] != NULL &&
	       !udc_dwc3_int_trb_hwo(&ep_data->trb_buf[tail]);
}

static bool udc_dwc3_sm_retire_tail(const struct device *dev,
				    struct udc_dwc3_ep_data *ep_data,
				    const char *via)
{
	return udc_dwc3_int_retire_sw_done(dev, ep_data, via);
}

static unsigned udc_dwc3_sm_poll_ep(const struct device *dev,
				    struct udc_dwc3_ep_data *ep_data)
{
	unsigned retired = 0U;

	if (ep_data->trb_buf == NULL || !udc_dwc3_ep_sm_is_cpu(ep_data)) {
		return 0U;
	}

	if (!udc_dwc3_sm_tail_retire_eligible(ep_data)) {
		ep_data->sm.poll_grace_armed = false;
		return 0U;
	}

	if (!ep_data->sm.poll_grace_armed || ep_data->sm.poll_grace_tail != ep_data->tail) {
		ep_data->sm.poll_grace_armed = true;
		ep_data->sm.poll_grace_tail = ep_data->tail;
		return 0U;
	}

	ep_data->sm.poll_grace_armed = false;
	while (udc_dwc3_sm_retire_tail(dev, ep_data, "poll")) {
		retired++;
	}

	return retired;
}

void udc_dwc3_ep_advance(const struct device *dev,
			 struct udc_dwc3_ep_data *ep_data,
			 enum udc_dwc3_ep_adv_reason reason)
{
	const char *via = udc_dwc3_sm_reason_str(reason);

	if (!udc_dwc3_ep_sm_is_cpu(ep_data) || ep_data->trb_buf == NULL) {
		return;
	}

	if ((reason == UDC_DWC3_EP_ADV_DEPEVT || reason == UDC_DWC3_EP_ADV_POLL) &&
	    !udc_dwc3_int_bulk_eps_live(dev)) {
#if defined(CONFIG_UDC_DWC3_EP_SM_LOG_PHASE)
		LOG_WRN("EP-SM: ADVANCE-BLOCKED ep=0x%02x via=%s live=0 state=%s "
			"active=%d",
			ep_data->cfg.addr, via, udc_dwc3_sm_state_str(ep_data->sm.state),
			ep_data->xfer_active);
#endif
		return;
	}

	if (ep_data->cfg.stat.halted &&
	    ep_data->sm.state != UDC_DWC3_EP_SM_CLEAR_PENDING) {
		return;
	}

	/* HWO cleared without event — SW retire (grace skipped for worker). */
	if (reason == UDC_DWC3_EP_ADV_WORKER) {
		while (udc_dwc3_sm_retire_tail(dev, ep_data, via)) {
			;
		}
		return;
	}

	/* Tail still HWO=1 on OUT run-dry hint — nudge UpdateXfer. */
	if (reason == UDC_DWC3_EP_ADV_DEPEVT &&
	    USB_EP_DIR_IS_OUT(ep_data->cfg.addr) && ep_data->xfer_active &&
	    ep_data->net_buf[ep_data->tail] != NULL &&
	    udc_dwc3_int_trb_hwo(&ep_data->trb_buf[ep_data->tail]) &&
	    udc_dwc3_int_ring_data_hwo_mask(ep_data) != 0U) {
		(void)udc_dwc3_doorbell_issue(dev, ep_data, UDC_DWC3_DB_UPDATE_VERIFY);
		udc_dwc3_int_submit_ep_work(ep_data);
		return;
	}

	/* Poll path uses grace gating inside poll_all. */
	if (reason == UDC_DWC3_EP_ADV_POLL) {
		(void)udc_dwc3_sm_poll_ep(dev, ep_data);
		return;
	}

	/*
	 * DEPEVT hint: retire every tail slot HW has released (HWO=0) with a
	 * matching net_buf.  Duplicate/late events find tail already advanced
	 * with NULL buf — handled in ep_sm_depevt() before we get here.
	 */
	if (reason == UDC_DWC3_EP_ADV_DEPEVT) {
		while (udc_dwc3_sm_retire_tail(dev, ep_data, via)) {
			;
		}
		udc_dwc3_int_submit_ep_work(ep_data);
	}
}

void udc_dwc3_ep_sm_reset_all(const struct device *dev)
{
	for (int i = 1; i < udc_dwc3_int_num_in_eps(dev); i++) {
		udc_dwc3_ep_sm_init(udc_dwc3_int_ep_in(dev, i));
	}
	for (int i = 1; i < udc_dwc3_int_num_out_eps(dev); i++) {
		udc_dwc3_ep_sm_init(udc_dwc3_int_ep_out(dev, i));
	}
}

unsigned udc_dwc3_ep_sm_poll_all(const struct device *dev)
{
	unsigned retired = 0U;

	if (!udc_dwc3_int_bulk_eps_live(dev)) {
		return 0U;
	}

	for (int i = 1; i < udc_dwc3_int_num_in_eps(dev); i++) {
		retired += udc_dwc3_sm_poll_ep(dev, udc_dwc3_int_ep_in(dev, i));
	}
	for (int i = 1; i < udc_dwc3_int_num_out_eps(dev); i++) {
		retired += udc_dwc3_sm_poll_ep(dev, udc_dwc3_int_ep_out(dev, i));
	}

	if (retired > 0U) {
		atomic_add(&udc_dwc3_sm_poll_recovered, retired);
	}

	{
		static int64_t last_log;
		static atomic_val_t last_recovered;
		static atomic_val_t last_retook;
		static atomic_val_t last_recycled;
		static atomic_val_t last_stuck;
		const atomic_val_t recovered = atomic_get(&udc_dwc3_sm_poll_recovered);
		const atomic_val_t retook = atomic_get(&udc_dwc3_sm_in_start_retook);
		const atomic_val_t recycled = atomic_get(&udc_dwc3_sm_in_start_recycled);
		const atomic_val_t stuck = atomic_get(&udc_dwc3_sm_in_start_exhausted);
		const int64_t now = k_uptime_get();

		if ((recovered != last_recovered || retook != last_retook ||
		     recycled != last_recycled || stuck != last_stuck) &&
		    (now - last_log >= 1000)) {
#if defined(CONFIG_UDC_DWC3_EP_SM_LOG_PHASE)
			LOG_WRN("ep-sm-recovery: lost-compl=%ld start-retook=%ld "
				"start-recycled=%ld start-stuck=%ld",
				(long)recovered, (long)retook, (long)recycled, (long)stuck);
#else
			LOG_DBG("ep-sm-recovery: lost-compl=%ld start-retook=%ld "
				"start-recycled=%ld start-stuck=%ld",
				(long)recovered, (long)retook, (long)recycled, (long)stuck);
#endif
			last_log = now;
			last_recovered = recovered;
			last_retook = retook;
			last_recycled = recycled;
			last_stuck = stuck;
		}
	}

	return retired;
}

bool udc_dwc3_ep_sm_depevt(const struct device *dev, uint32_t evt)
{
	struct udc_dwc3_ep_data *ep_data = udc_dwc3_int_ep_from_evt(dev, evt);

	if (!udc_dwc3_int_bulk_eps_live(dev)) {
#if defined(CONFIG_UDC_DWC3_EP_SM_LOG_PHASE)
		if (udc_dwc3_ep_sm_is_cpu(ep_data)) {
			LOG_WRN("EP-SM: DEPEVT-PREMATURE ep=0x%02x evt=0x%08x live=0",
				ep_data->cfg.addr, evt);
		}
#endif
		return false;
	}

	if (!udc_dwc3_ep_sm_is_cpu(ep_data)) {
		return false;
	}

	if (!ep_data->xfer_active && ep_data->sm.state == UDC_DWC3_EP_SM_IDLE) {
#if defined(CONFIG_UDC_DWC3_EP_SM_LOG_PHASE)
		LOG_DBG("EP-SM: DEPEVT-IDLE-SKIP ep=0x%02x evt=0x%08x",
			ep_data->cfg.addr, evt);
#endif
		return false;
	}

	const bool from_inprog = ((evt & GENMASK(7, 6)) == (0x2U << 6));
	const uint32_t tail = ep_data->tail;
	const bool hwo = udc_dwc3_int_trb_hwo(&ep_data->trb_buf[tail]);
	const bool had_buf = ep_data->net_buf[tail] != NULL;

	if (ep_data->skip_xfer_done_count > 0U) {
		return false;
	}

	/*
	 * HWO-first: DEPEVT does not name a TRB slot — tail is SW's best guess.
	 * If tail has no net_buf and HWO is clear, HW and SW already agree the
	 * slot was retired (late/duplicate hint after poll or a prior completion).
	 */
	if (!had_buf && !hwo) {
		LOG_DBG("EP-SM: DEPEVT-DUP-HINT ep=0x%02x evt=0x%08x inprog=%d "
			"tail=%u",
			ep_data->cfg.addr, evt, from_inprog, tail);
		udc_dwc3_int_submit_ep_work(ep_data);
		return true;
	}

#if defined(CONFIG_UDC_DWC3_EP_SM_LOG_UNMATCHED)
	if (!had_buf) {
		LOG_WRN("EP-SM: UNMATCHED-DEPEVT ep=0x%02x evt=0x%08x inprog=%d "
			"state=%d tail=%u hwo=%d",
			ep_data->cfg.addr, evt, from_inprog, ep_data->sm.state,
			tail, hwo);
	} else if (hwo && !from_inprog) {
		LOG_WRN("EP-SM: DEPEVT-HWO ep=0x%02x evt=0x%08x state=%d tail=%u",
			ep_data->cfg.addr, evt, ep_data->sm.state, tail);
	}
#endif

	udc_dwc3_ep_advance(dev, ep_data, UDC_DWC3_EP_ADV_DEPEVT);

	/* Tail still HWO=1 with a buffer: norm handles DONE-HWO defer. */
	if (had_buf && hwo) {
		return false;
	}

	return true;
}
