/*
 * SPDX-FileCopyrightText: Copyright tinyVision.ai Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * HWO-ground-truth state machine for CPU-filled bulk endpoints (ACM, etc.).
 * Endpoints 0x83 and 0x84 are excluded by address.
 */

#ifndef ZEPHYR_DRIVERS_USB_UDC_DWC3_EP_SM_H_
#define ZEPHYR_DRIVERS_USB_UDC_DWC3_EP_SM_H_

#include <stdint.h>
#include <stdbool.h>

#include <zephyr/sys/atomic.h>

struct device;
struct udc_dwc3_ep_data;
struct udc_dwc3_config;
struct udc_dwc3_ep_sm;

/** Per-CPU-EP transfer-resource lifecycle (HWO is separate ground truth). */
enum udc_dwc3_ep_sm_state {
	UDC_DWC3_EP_SM_IDLE = 0,
	UDC_DWC3_EP_SM_ACTIVE,
	UDC_DWC3_EP_SM_HALTED,
	UDC_DWC3_EP_SM_CLEAR_PENDING,
};

/** Why ep_advance() was invoked — events are hints; HWO decides action. */
enum udc_dwc3_ep_adv_reason {
	UDC_DWC3_EP_ADV_DEPEVT = 0,
	UDC_DWC3_EP_ADV_POLL,
	UDC_DWC3_EP_ADV_DOORBELL,
	UDC_DWC3_EP_ADV_WORKER,
};

enum udc_dwc3_doorbell_cmd {
	UDC_DWC3_DB_START = 0,
	UDC_DWC3_DB_UPDATE,
	/** OUT run-dry / park: UpdateXfer + HWO fetch verify. */
	UDC_DWC3_DB_UPDATE_VERIFY,
	/** IN resume into a parked ring: UpdateXfer + HWO fetch verify. */
	UDC_DWC3_DB_UPDATE_VERIFY_IN,
};

#if defined(CONFIG_UDC_DWC3_EP_SM)

bool udc_dwc3_ep_sm_is_cpu(const struct udc_dwc3_ep_data *ep_data);
void udc_dwc3_ep_sm_init(struct udc_dwc3_ep_data *ep_data);
void udc_dwc3_ep_sm_set_state(struct udc_dwc3_ep_data *ep_data,
			      enum udc_dwc3_ep_sm_state state);

int udc_dwc3_doorbell_issue(const struct device *dev,
			    struct udc_dwc3_ep_data *ep_data,
			    enum udc_dwc3_doorbell_cmd cmd);

void udc_dwc3_ep_advance(const struct device *dev,
			 struct udc_dwc3_ep_data *ep_data,
			 enum udc_dwc3_ep_adv_reason reason);

unsigned udc_dwc3_ep_sm_poll_all(const struct device *dev);

/** Tail-progress watchdog: report, and recover, endpoints that stop retiring. */
void udc_dwc3_ep_sm_watchdog(const struct device *dev);

/** True if any CPU endpoint still holds a queued buffer. */
bool udc_dwc3_ep_sm_any_pending(const struct device *dev);

void udc_dwc3_ep_sm_reset_all(const struct device *dev);

/** DEPEVT fast-path for CPU bulk eps; returns false if caller should handle. */
bool udc_dwc3_ep_sm_depevt(const struct device *dev, uint32_t evt);

struct udc_dwc3_in_recovery_stats {
	atomic_val_t poll_recovered;
	atomic_val_t start_retook;
	atomic_val_t start_backoff;
	atomic_val_t start_recycled;
	atomic_val_t start_stuck;
};

void udc_dwc3_ep_sm_in_recovery_get(struct udc_dwc3_in_recovery_stats *stats);

#else /* !CONFIG_UDC_DWC3_EP_SM */

static inline bool udc_dwc3_ep_sm_is_cpu(const struct udc_dwc3_ep_data *ep_data)
{
	ARG_UNUSED(ep_data);
	return false;
}

static inline void udc_dwc3_ep_sm_init(struct udc_dwc3_ep_data *ep_data)
{
	ARG_UNUSED(ep_data);
}

static inline void udc_dwc3_ep_sm_set_state(struct udc_dwc3_ep_data *ep_data,
					    enum udc_dwc3_ep_sm_state state)
{
	ARG_UNUSED(ep_data);
	ARG_UNUSED(state);
}

static inline int udc_dwc3_doorbell_issue(const struct device *dev,
					 struct udc_dwc3_ep_data *ep_data,
					 enum udc_dwc3_doorbell_cmd cmd)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(ep_data);
	ARG_UNUSED(cmd);
	return 0;
}

static inline void udc_dwc3_ep_advance(const struct device *dev,
				     struct udc_dwc3_ep_data *ep_data,
				     enum udc_dwc3_ep_adv_reason reason)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(ep_data);
	ARG_UNUSED(reason);
}

static inline unsigned udc_dwc3_ep_sm_poll_all(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0U;
}

static inline void udc_dwc3_ep_sm_watchdog(const struct device *dev)
{
	ARG_UNUSED(dev);
}

static inline bool udc_dwc3_ep_sm_any_pending(const struct device *dev)
{
	ARG_UNUSED(dev);
	return false;
}

static inline bool udc_dwc3_ep_sm_depevt(const struct device *dev, uint32_t evt)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(evt);
	return false;
}

#endif /* CONFIG_UDC_DWC3_EP_SM */

#endif /* ZEPHYR_DRIVERS_USB_UDC_DWC3_EP_SM_H_ */
