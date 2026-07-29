/*
 * SPDX-FileCopyrightText: Copyright tinyVision.ai Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Internal cross-file hooks between udc_dwc3.c and udc_dwc3_ep_sm.c.
 */

#ifndef ZEPHYR_DRIVERS_USB_UDC_DWC3_INT_H_
#define ZEPHYR_DRIVERS_USB_UDC_DWC3_INT_H_

#include <zephyr/kernel.h>
#include <zephyr/drivers/usb/udc.h>
#include <zephyr/net_buf.h>

struct device;
struct udc_dwc3_config;

struct udc_dwc3_trb {
	uint32_t addr_lo;
	uint32_t addr_hi;
	uint32_t status;
	uint32_t ctrl;
} __packed __aligned(16);

#include "udc_dwc3_ep_sm.h"

/* SM-owned fields appended to ep_data (single definition for udc_dwc3.c + ep_sm.c). */
struct udc_dwc3_ep_sm {
	enum udc_dwc3_ep_sm_state state;
	bool out_rundry_reported;
	bool in_start_reported;
	/** Set while sm_in_start_verify runs (blocks concurrent poll retire races). */
	bool in_start_verify_busy;
	/** Set during tier-5 IN ring nuke (blocks poll SW-retire on same buf). */
	bool tier5_recovering;
#if defined(CONFIG_UDC_DWC3_IN_COMPLETION_POLL) || defined(CONFIG_UDC_DWC3_EP_SM)
	uint32_t poll_grace_tail;
	bool poll_grace_armed;
#endif
	/** Tail-progress watchdog, see udc_dwc3_ep_sm_watchdog(). */
	int64_t stall_since;
	uint32_t stall_tail;
	struct net_buf *stall_buf;
	/** Bytes the controller still owed on the tail TRB when the window opened. */
	uint32_t stall_remaining;
	bool stall_reported;
	/** Consecutive OUT doorbell refreshes with no progress. */
	uint8_t out_refresh_count;
};

/*
 * All data specific to one endpoint for use by the driver.
 * Must match the instance embedded in udc_dwc3.c DEVICE_DT_INST_DEFINE arrays.
 */
struct udc_dwc3_ep_data {
	/* Allow to cast a pointer between ep_data and ep_cfg */
	struct udc_ep_config cfg;
	/* Endpoint number (physical address): the logical address is on ep_cfg */
	int epn;
	/* A work queue entry to process the buffers to submit on that endpoint */
	struct k_work work;
	/* Point back to the device for work queues */
	const struct device *dev;
	/* Buffer of pointers to net_buf, with index matching the position in the TRB buffers */
	struct net_buf *net_buf[CONFIG_UDC_DWC3_TRB_NUM];
	/* Buffer of TRB structures, with index matching the position in the net_buf buffers */
	struct udc_dwc3_trb *trb_buf;
	/* Index of the next TRB to receive data in the TRB ring, Link TRB excluded */
	uint32_t head;
	uint32_t tail;
	/* Total size sent from the device to the host for the ongoing transfer */
	uint32_t total;
	/* A flag to tell when the ring buffer is full */
	bool full;
	/* Given by the hardware for use in endpoint commands */
	uint32_t xferrscidx;
	/* MPS-aligned IN data TRB held until chained internal ZLP completes */
	struct net_buf *chain_buf;
	/* Next CDC ACM len==0 enqueue is absorbed (ZLP already sent on wire) */
	bool absorb_cdc_zlp;
	/* False after an LST-terminated IN transfer until DepStartXfer re-arms the EP */
	bool xfer_active;
	/* XFERCOMPLETE events to ignore after try_retire_chained_zlp() popped the ZLP */
	uint8_t skip_xfer_done_count;
#if defined(CONFIG_UDC_DWC3_IN_START_ENDXFER_ESCALATE)
	/* Buffers re-queued by tier-5 recovery, bounding retries on a dead endpoint */
	uint8_t tier5_requeues;
#endif
#if defined(CONFIG_UDC_DWC3_EP_SM)
	struct udc_dwc3_ep_sm sm;
#elif defined(CONFIG_UDC_DWC3_IN_COMPLETION_POLL)
	uint32_t poll_grace_tail;
	bool poll_grace_armed;
#endif
};

bool udc_dwc3_int_trb_hwo(const volatile struct udc_dwc3_trb *trb);
uint32_t udc_dwc3_int_ring_data_hwo_mask(const struct udc_dwc3_ep_data *ep_data);

/** Bytes the controller has not yet consumed from @p trb. */
uint32_t udc_dwc3_int_trb_remaining(const struct udc_dwc3_trb *trb);

uint32_t udc_dwc3_int_depcmd_issue(const struct device *dev, uint32_t depcmd_addr,
				   uint32_t cmd, bool *cmderr);

void udc_dwc3_int_depcmd_start_xfer(const struct device *dev,
				    struct udc_dwc3_ep_data *ep_data);

void udc_dwc3_int_depcmd_start_xfer_trb(const struct device *dev,
					struct udc_dwc3_ep_data *ep_data,
					struct udc_dwc3_trb *trb);

void udc_dwc3_int_depcmd_update_xfer(const struct device *dev,
				     struct udc_dwc3_ep_data *ep_data);

uint32_t udc_dwc3_int_depcmd_update_xfer_checked(const struct device *dev,
						 struct udc_dwc3_ep_data *ep_data,
						 bool *cmderr);

atomic_val_t udc_dwc3_int_in_start_recycled_get(void);
atomic_val_t udc_dwc3_int_in_start_exhausted_get(void);

void udc_dwc3_int_on_xfer_done_norm(const struct device *dev, uint32_t evt);

bool udc_dwc3_int_retire_sw_done(const struct device *dev,
				 struct udc_dwc3_ep_data *ep_data,
				 const char *via);

void udc_dwc3_int_in_endxfer_recycle(const struct device *dev,
				     struct udc_dwc3_ep_data *ep_data);

bool udc_dwc3_int_out_endxfer_recycle(const struct device *dev,
				      struct udc_dwc3_ep_data *ep_data);

void udc_dwc3_int_submit_ep_work(struct udc_dwc3_ep_data *ep_data);

struct udc_dwc3_ep_data *udc_dwc3_int_ep_from_evt(const struct device *dev, uint32_t evt);

int udc_dwc3_int_num_in_eps(const struct device *dev);
int udc_dwc3_int_num_out_eps(const struct device *dev);
struct udc_dwc3_ep_data *udc_dwc3_int_ep_in(const struct device *dev, int idx);
struct udc_dwc3_ep_data *udc_dwc3_int_ep_out(const struct device *dev, int idx);

#if defined(CONFIG_UDC_DWC3_EP_SM)
bool udc_dwc3_int_bulk_eps_live(const struct device *dev);
void udc_dwc3_ep_sm_reset_all(const struct device *dev);
#endif

#endif /* ZEPHYR_DRIVERS_USB_UDC_DWC3_INT_H_ */
