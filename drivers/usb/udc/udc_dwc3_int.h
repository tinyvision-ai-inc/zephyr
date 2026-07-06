/*
 * SPDX-FileCopyrightText: Copyright tinyVision.ai Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Internal cross-file hooks between udc_dwc3.c and udc_dwc3_ep_sm.c.
 */

#ifndef ZEPHYR_DRIVERS_USB_UDC_DWC3_INT_H_
#define ZEPHYR_DRIVERS_USB_UDC_DWC3_INT_H_

#include <zephyr/drivers/usb/udc.h>
#include <zephyr/net_buf.h>

struct device;
struct udc_dwc3_ep_data;
struct udc_dwc3_config;

struct udc_dwc3_trb {
	uint32_t addr_lo;
	uint32_t addr_hi;
	uint32_t status;
	uint32_t ctrl;
} __packed __aligned(16);

#include "udc_dwc3_ep_sm.h"

/* SM-owned fields appended to ep_data (see udc_dwc3.c). */
struct udc_dwc3_ep_sm {
	enum udc_dwc3_ep_sm_state state;
#if defined(CONFIG_UDC_DWC3_IN_COMPLETION_POLL) || defined(CONFIG_UDC_DWC3_EP_SM)
	uint32_t poll_grace_tail;
	bool poll_grace_armed;
#endif
};

bool udc_dwc3_int_trb_hwo(const volatile struct udc_dwc3_trb *trb);
uint32_t udc_dwc3_int_ring_data_hwo_mask(const struct udc_dwc3_ep_data *ep_data);

uint32_t udc_dwc3_int_depcmd_issue(const struct device *dev, uint32_t depcmd_addr,
				   uint32_t cmd, bool *cmderr);

void udc_dwc3_int_depcmd_start_xfer(const struct device *dev,
				    struct udc_dwc3_ep_data *ep_data);

void udc_dwc3_int_depcmd_start_xfer_trb(const struct device *dev,
					struct udc_dwc3_ep_data *ep_data,
					struct udc_dwc3_trb *trb);

void udc_dwc3_int_depcmd_update_xfer(const struct device *dev,
				     struct udc_dwc3_ep_data *ep_data);

void udc_dwc3_int_on_xfer_done_norm(const struct device *dev, uint32_t evt);

bool udc_dwc3_int_retire_sw_done(const struct device *dev,
				 struct udc_dwc3_ep_data *ep_data,
				 const char *via);

void udc_dwc3_int_in_endxfer_recycle(const struct device *dev,
				     struct udc_dwc3_ep_data *ep_data);

void udc_dwc3_int_submit_ep_work(struct udc_dwc3_ep_data *ep_data);

struct udc_dwc3_ep_data *udc_dwc3_int_ep_from_evt(const struct device *dev, uint32_t evt);

const struct udc_dwc3_config *udc_dwc3_int_cfg(const struct device *dev);

#endif /* ZEPHYR_DRIVERS_USB_UDC_DWC3_INT_H_ */
