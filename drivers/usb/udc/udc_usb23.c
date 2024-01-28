/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2023 tinyVision.ai
 *
 * Driver for the USB23 physical core as found on Lattice CrosslinkU-NX FPGAs.
 * https://www.latticesemi.com/products/designsoftwareandip/intellectualproperty/ipcore/ipcores05/usb-2_0-3_2-ip-core
 *
 * Being an FPGA, a soft CPU core such as a RISC-V running Zephyr must be
 * present in the FPGA hardware design. This driver is to be run on it.
 */

#define DT_DRV_COMPAT lattice_usb23

/*
 * USB device controller (UDC) for Lattice USB2 and USB3 hardened FPGA core.
 */
#include "udc_common.h"
#include "udc_usb23.h"
#include "vexriscv.h"

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/usb/udc.h>
#include <zephyr/sys/util.h>
#include <app_version.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(usb23, CONFIG_UDC_DRIVER_LOG_LEVEL);

#define LO32(n) ((uint32_t)((uint64_t)(n) & 0xffffffff))
#define HI32(n) ((uint32_t)((uint64_t)(n) >> 32))
#define U64(a, b) (((uint64_t)(a)) << 32 | (((uint64_t)(b)) & 0xffffffff))
#define MEM32(addr) (*(volatile uint32_t *)(addr))
#define CASE(label, ...) case label: LOG_INF(#label ": " __VA_ARGS__)
#define GETFIELD(reg, prefix) ((reg & prefix##_MASK) >> prefix##_SHIFT)

/*
 * Structure for holding controller configuration items that can remain in
 * non-volatile memory.
 */
struct usb23_config {
	/* USB endpoints configuration */
	size_t num_of_eps;
	struct udc_ep_config *ep_cfg_in;
	struct udc_ep_config *ep_cfg_out;

	/* USB device configuration */
	int speed_idx;

	/* Base address at which is mapped the USB23 core */
	uintptr_t base;

	/* Functions pointers for managing the IRQs from LiteX */
	void (*irq_enable_func)(void);
	void (*irq_clear_func)(void);
};

/*
 * All data specific to one endpoint
 */
struct usb23_ep_data {
	/* TRB buffer, memory shared with the USB23 core */
	volatile struct usb23_trb trb[32] __attribute__((__aligned__(16)));

	/* UDC buffer currently being working on */
	struct net_buf *buf;

	/* Given by the hardware for use in endpoint commands */
	uint32_t xferrscidx;
};

/*
 * Data of each instance of the driver, that can be read and written to.
 * Accessed via "udc_get_private(dev)".
 */
struct usb23_data {
	/* Buffer passed to the USB23 core who fills it with event information */
	volatile union usb23_evt evt_buf[128] __attribute__((__aligned__(16)));

	/* Index within trb where to queue new TRBs */
	uint8_t evt_next;

	/* As many items as there are endpoints in the system */
	struct usb23_ep_data *ep_data;

	/* A work queue entry to process the events from the event buffer */
	struct k_work work;

	/* Back-reference to parent */
	const struct device *dev;
};

struct usb23_reg {
	uint32_t addr;
	char *name;
	uint32_t last;
};

static int usb23_set_address(const struct device *dev, const uint8_t addr);
static int usb23_ep_enqueue(const struct device *dev,
		struct udc_ep_config *const ep_cfg, struct net_buf *buf);
struct usb23_reg *usb23_get_reg(uint32_t addr);

/*------------------------------------------------------------------------------
--  Input/Output
--------------------------------------------------------------------------------
--  Communication with the USB23 register interface through memory addresses
--  The USB23 core has a register interface and a DMA interface memory.
--  These helpers are the only places where I/O is done with it.
*/

static uint32_t usb23_io_read(const struct device *dev, uint32_t addr)
{
	const struct usb23_config *config = dev->config;

	return MEM32(config->base + addr);
}

static void usb23_io_write(const struct device *dev, uint32_t addr,
		uint32_t data)
{
	const struct usb23_config *config = dev->config;

	MEM32(config->base + addr) = data;
}

static void usb23_io_wait_go_low(const struct device *dev, uint32_t addr,
		uint32_t mask)
{
	while ((usb23_io_read(dev, addr) & mask) != 0);
}

static void usb23_io_set(const struct device *dev, uint32_t addr,
		uint32_t mask)
{
	usb23_io_write(dev, addr, usb23_io_read(dev, addr) | mask);
}

static void usb23_io_clear(const struct device *dev, uint32_t addr,
		uint32_t mask)
{
	usb23_io_write(dev, addr, usb23_io_read(dev, addr) & ~mask);
}

static void usb23_io_field(const struct device *dev, uint32_t addr,
		uint32_t mask, uint32_t val)
{
	usb23_io_write(dev, addr, (usb23_io_read(dev, addr) & ~mask) | val);
}

/*------------------------------------------------------------------------------
--  Getters
--------------------------------------------------------------------------------
--  Helpers to convert between various numbers formats or accessing one struct
--  from another. No action on the hardware.
*/

/*
 * This is a mapping between logical and physical resources we decoreed.
 * Convert from USB standard endpoint number to physical resource number.
 * It alternates between OUT endpoints (0x00) and IN endpoints (0x80).
 * From: 0x00, 0x80, 0x01, 0x81, 0x02, 0x82, 0x03, 0x83, ...
 * To:   0,    1,    2,    3,    4,    5,    6,    7,    ...
 */
static int usb23_get_ep_physical_num(struct udc_ep_config *const ep_cfg)
{
	return ((ep_cfg->addr & 0b01111111) << 1) | (ep_cfg->addr >> 7);
}

/*
 * Control EP must have the same value for IN and OUT EP. This field should be
 * set to 0 for all other OUT EPs. Device mode must use lower 16 TxFIFOs.
 */
static int usb23_get_ep_fifo_num(struct udc_ep_config *const ep_cfg)
{
	if ((ep_cfg->addr & 0x80) == 0x00) {
		/* OUT endpoints always get FIFO 0 */
		return 0;
	} else {
		/* One FIFO per IN endpoint, starting with 0 for endpoint 0 */
		return ep_cfg->addr & ~0x80;
	}
}

static struct udc_ep_config *const usb23_get_ep_cfg(const struct device *dev,
		int epn)
{
	return udc_get_ep_cfg(dev, ((epn & 1) << 7) | (epn >> 1));
}

static struct usb23_ep_data *usb23_get_ep_data(const struct device *dev,
		struct udc_ep_config *const ep_cfg)
{
	const struct usb23_config *config = dev->config;
	struct usb23_data *priv = udc_get_private(dev);
	int epn = usb23_get_ep_physical_num(ep_cfg);

	__ASSERT_NO_MSG(epn < config->num_of_eps * 2);
	return &priv->ep_data[epn];
}

static struct usb23_trb usb23_get_trb(const struct device *dev,
		struct udc_ep_config *const ep_cfg)
{
	flush_l2_cache();
	flush_cpu_dcache();
	return *usb23_get_ep_data(dev, ep_cfg)->trb;
}

static void usb23_set_trb(const struct device *dev,
		struct udc_ep_config *const ep_cfg, struct usb23_trb *trb)
{
	*usb23_get_ep_data(dev, ep_cfg)->trb = *trb;
	flush_cpu_dcache();
	flush_l2_cache();
}

static union usb23_evt usb23_get_next_evt(const struct device *dev)
{
	struct usb23_data *priv = udc_get_private(dev);
	volatile union usb23_evt evt;

	/* Cache the current event */
	flush_l2_cache();
	flush_cpu_dcache();
	evt = priv->evt_buf[priv->evt_next];

	/* Clear it on the event buffer */
	priv->evt_buf[priv->evt_next].raw = 0x00000000;
	flush_l2_cache();
	flush_cpu_dcache();

	/* This is a ring buffer, wrap around */
	priv->evt_next++;
	priv->evt_next %= ARRAY_SIZE(priv->evt_buf);

	return evt;
}

/*------------------------------------------------------------------------------
--  Debug
--------------------------------------------------------------------------------
--  Temporary functions used for debugging, meant to be removed once the
--  implementation is complete.
*/

struct usb23_reg usb23_regs[] = {
#define	R(reg) { .addr = USB23_##reg, .name = #reg, .last = 0 }

	/* main registers */
	R(GCTL),
	R(DCTL),
	R(DCFG),
	R(DEVTEN),
	R(DALEPENA),
	R(GCOREID),
	R(GSTS),
	R(DSTS),
	R(GEVNTADR_LO(0)), R(GEVNTADR_HI(0)),
	R(GEVNTSIZ(0)),
	R(GEVNTCOUNT(0)),
	R(GUSB2PHYCFG),
	R(GUSB3PIPECTL),

	/* debug */
	R(GBUSERRADDR_LO), R(GBUSERRADDR_HI),
	R(CTLDEBUG_LO), R(CTLDEBUG_HI),
	R(ANALYZERTRACE),
	R(GDBGFIFOSPACE),
	R(GDBGLTSSM),
	R(GDBGLNMCC),
	R(GDBGBMU),
	R(GDBGLSPMUX_DEV),
	R(GDBGLSPMUX_HST),
	R(GDBGLSP),
	R(GDBGEPINFO0), R(GDBGEPINFO1),
	R(BU3RHBDBG0),

	/* undocumented registers from Lattice */
	R(U2PHYCTRL0), R(U2PHYCTRL1), R(U2PHYCTRL2),
	R(U3PHYCTRL0), R(U3PHYCTRL1), R(U3PHYCTRL2), R(U3PHYCTRL3), R(U3PHYCTRL4), R(U3PHYCTRL5),

	/* physical endpoint numbers */
	R(DEPCMDPAR2(0)), R(DEPCMDPAR1(0)), R(DEPCMDPAR0(0)), R(DEPCMD(0)),
	R(DEPCMDPAR2(1)), R(DEPCMDPAR1(1)), R(DEPCMDPAR0(1)), R(DEPCMD(1)),
	R(DEPCMDPAR2(2)), R(DEPCMDPAR1(2)), R(DEPCMDPAR0(2)), R(DEPCMD(2)),
	R(DEPCMDPAR2(3)), R(DEPCMDPAR1(3)), R(DEPCMDPAR0(3)), R(DEPCMD(3)),
	R(DEPCMDPAR2(4)), R(DEPCMDPAR1(4)), R(DEPCMDPAR0(4)), R(DEPCMD(4)),
	R(DEPCMDPAR2(5)), R(DEPCMDPAR1(5)), R(DEPCMDPAR0(5)), R(DEPCMD(5)),
	R(DEPCMDPAR2(6)), R(DEPCMDPAR1(6)), R(DEPCMDPAR0(6)), R(DEPCMD(6)),
	R(DEPCMDPAR2(7)), R(DEPCMDPAR1(7)), R(DEPCMDPAR0(7)), R(DEPCMD(7)),

	/* Hardware parameters */
	R(GHWPARAMS0), R(GHWPARAMS1), R(GHWPARAMS2), R(GHWPARAMS3),
	R(GHWPARAMS4), R(GHWPARAMS5), R(GHWPARAMS6), R(GHWPARAMS7),
	R(GHWPARAMS8),
};

struct usb23_reg *usb23_get_reg(uint32_t addr)
{
	for (int i = 0; i < ARRAY_SIZE(usb23_regs); i++) {
		struct usb23_reg *ureg = &usb23_regs[i];

		if (ureg->addr == addr) {
			return ureg;
		}
	}
	return NULL;
}

void usb23_dump_registers(const struct device *dev)
{
	LOG_INF("%s", __func__);
	for (int i = 0; i < ARRAY_SIZE(usb23_regs); i++) {
		struct usb23_reg *ureg = &usb23_regs[i];
		uint32_t data;

		data = usb23_io_read(dev, ureg->addr);
		if (data != ureg->last) {
			LOG_INF("\treg 0x%08x == 0x%08x  (was 0x%08x)  // %s",
				ureg->addr, data, ureg->last, ureg->name);
			ureg->last = data;
		}
	}
}

void usb23_dump_link_state(const struct device *dev)
{
	uint32_t reg;

	reg = usb23_io_read(dev, USB23_DSTS);

	switch (reg & USB23_DSTS_CONNECTSPD_MASK) {
	case USB23_DSTS_CONNECTSPD_HS:
	case USB23_DSTS_CONNECTSPD_FS:
		switch (reg & USB23_DSTS_USBLNKST_MASK) {
		CASE(USB23_DSTS_USBLNKST_USB2_ON_STATE); break;
		CASE(USB23_DSTS_USBLNKST_USB2_SLEEP_STATE); break;
		CASE(USB23_DSTS_USBLNKST_USB2_SUSPEND_STATE); break;
		CASE(USB23_DSTS_USBLNKST_USB2_DISCONNECTED); break;
		CASE(USB23_DSTS_USBLNKST_USB2_EARLY_SUSPEND); break;
		CASE(USB23_DSTS_USBLNKST_USB2_RESET); break;
		CASE(USB23_DSTS_USBLNKST_USB2_RESUME); break;
		}
		break;
	case USB23_DSTS_CONNECTSPD_SS:
		switch (reg & USB23_DSTS_USBLNKST_MASK) {
		CASE(USB23_DSTS_USBLNKST_USB3_U0); break;
		CASE(USB23_DSTS_USBLNKST_USB3_U1); break;
		CASE(USB23_DSTS_USBLNKST_USB3_U2); break;
		CASE(USB23_DSTS_USBLNKST_USB3_U3); break;
		CASE(USB23_DSTS_USBLNKST_USB3_SS_DIS); break;
		CASE(USB23_DSTS_USBLNKST_USB3_RX_DET); break;
		CASE(USB23_DSTS_USBLNKST_USB3_SS_INACT); break;
		CASE(USB23_DSTS_USBLNKST_USB3_POLL); break;
		CASE(USB23_DSTS_USBLNKST_USB3_RECOV); break;
		CASE(USB23_DSTS_USBLNKST_USB3_HRESET); break;
		CASE(USB23_DSTS_USBLNKST_USB3_CMPLY); break;
		CASE(USB23_DSTS_USBLNKST_USB3_LPBK); break;
		CASE(USB23_DSTS_USBLNKST_USB3_RESET_RESUME); break;
		}
		break;
	default:
		LOG_INF("unknown speed"); return;
	}
}

void usb23_dump_events(const struct device *dev)
{
	struct usb23_data *priv = udc_get_private(dev);

	for (int i = 0; i < ARRAY_SIZE(priv->evt_buf); i++) {
		volatile union usb23_evt evt = priv->evt_buf[i];
		char *s = (i == priv->evt_next) ? " <-" : "";

		if (evt.raw != 0x00000000) {
			LOG_DBG("\tevt 0x%02x: 0x%08x%s", i, evt.raw, s);
		}
	}
}

void usb23_dump_trbs(const struct device *dev)
{
	const struct usb23_config *config = dev->config;

	for (int i = 0; i < config->num_of_eps; i++) {
		struct udc_ep_config *ep_cfg;
		struct usb23_trb trb;

		ep_cfg = udc_get_ep_cfg(dev, USB_EP_DIR_OUT | i);
		trb = usb23_get_trb(dev, ep_cfg);
		LOG_DBG("EPx%02x: addr=0x%08x%08x stat=0x%08x ctrl=0x%08x",
				ep_cfg->addr, trb.addr_hi, trb.addr_lo,
				trb.status, trb.ctrl);

		ep_cfg = udc_get_ep_cfg(dev, USB_EP_DIR_IN | i);
		trb = usb23_get_trb(dev, ep_cfg);
		LOG_DBG("EPx%02x: addr=0x%08x%08x stat=0x%08x ctrl=0x%08x",
				ep_cfg->addr, trb.addr_hi, trb.addr_lo,
				trb.status, trb.ctrl);
	}
}

/*------------------------------------------------------------------------------
--  Commands
--------------------------------------------------------------------------------
--  The DEPCMD register acts as a command interface, where a command number is
--  written along with parameters, an action is performed and a CMDACT bit is
--  reset whenever the command completes.
*/

static uint32_t usb23_cmd(const struct device *dev, uint32_t addr,
		uint32_t cmd)
{
	uint32_t reg;

	usb23_io_write(dev, addr, cmd | USB23_DEPCMD_CMDACT);
	do {
		reg = usb23_io_read(dev, addr);
	} while ((reg & USB23_DEPCMD_CMDACT) != 0);

	switch (reg & USB23_DEPCMD_STATUS_MASK) {
	case USB23_DEPCMD_STATUS_OK:
		break;
	case USB23_DEPCMD_STATUS_CMDERR:
		LOG_ERR("Command failed");
		break;
	default:
		LOG_ERR("Command failed with unknown status: 0x%08x", reg);
	}

	return GETFIELD(reg, USB23_DEPCMD_XFERRSCIDX);
}

static void usb23_cmd_ep_config(const struct device *dev,
		struct udc_ep_config *const ep_cfg)
{
	int epn = usb23_get_ep_physical_num(ep_cfg);
	uint32_t reg0 = 0, reg1 = 0;

	if (ep_cfg->stat.enabled) {
		reg0 |= USB23_DEPCMDPAR0_DEPCFG_ACTION_MODIFY;
	} else {
		reg0 |= USB23_DEPCMDPAR0_DEPCFG_ACTION_INIT;
	}

	switch (ep_cfg->attributes & USB_EP_TRANSFER_TYPE_MASK) {
	case USB_EP_TYPE_CONTROL:
		reg0 |= USB23_DEPCMDPAR0_DEPCFG_EPTYPE_CTRL;
		break;
	case USB_EP_TYPE_BULK:
		reg0 |= USB23_DEPCMDPAR0_DEPCFG_EPTYPE_BULK;
		break;
	case USB_EP_TYPE_INTERRUPT:
		reg0 |= USB23_DEPCMDPAR0_DEPCFG_EPTYPE_INT;
		break;
	case USB_EP_TYPE_ISO:
		reg0 |= USB23_DEPCMDPAR0_DEPCFG_EPTYPE_ISOC;
		break;
	default:
		__ASSERT_NO_MSG(false);
	}

	/* Max Packet Size according to the USB descriptor configuration */
	reg0 |= ep_cfg->mps << USB23_DEPCMDPAR0_DEPCFG_MPS_SHIFT;

	/* One-to-one mapping between FIFO numbers and endpoints numbers */
	reg0 |= usb23_get_ep_fifo_num(ep_cfg)
			<< USB23_DEPCMDPAR0_DEPCFG_FIFONUM_SHIFT;

	/* Per-endpoint events */
	reg1 |= USB23_DEPCMDPAR1_DEPCFG_XFERINPROGEN;
	reg1 |= USB23_DEPCMDPAR1_DEPCFG_XFERNRDYEN;
	reg1 |= USB23_DEPCMDPAR1_DEPCFG_XFERCMPLEN;

	/* This is the usb protocol endpoint number, but the data encoding
	 * we chose for physical endpoint number is the same as this register */
	reg1 |= epn << USB23_DEPCMDPAR1_DEPCFG_EPNUMBER_SHIFT;

	usb23_io_write(dev, USB23_DEPCMDPAR0(epn), reg0);
	usb23_io_write(dev, USB23_DEPCMDPAR1(epn), reg1);
	usb23_cmd(dev, USB23_DEPCMD(epn), USB23_DEPCMD_DEPCFG);
}

static void usb23_cmd_ep_xfer_config(const struct device *dev,
		struct udc_ep_config *const ep_cfg)
{
	int epn = usb23_get_ep_physical_num(ep_cfg);

	usb23_io_write(dev, USB23_DEPCMDPAR1(epn), 0x00000402); // TODO decode
	usb23_io_write(dev, USB23_DEPCMDPAR0(epn),
			1 << USB23_DEPCMD_DEPXFERCFG_NUMXFERRES_SHIFT);
	usb23_cmd(dev, USB23_DEPCMD(epn), USB23_DEPCMD_DEPXFERCFG);
}

static void usb23_cmd_set_stall(const struct device *dev,
		struct udc_ep_config *const ep_cfg)
{
	int epn = usb23_get_ep_physical_num(ep_cfg);

	usb23_cmd(dev, USB23_DEPCMD(epn), USB23_DEPCMD_DEPSETSTALL);
}

static void usb23_cmd_clear_stall(const struct device *dev,
		struct udc_ep_config *const ep_cfg)
{
	int epn = usb23_get_ep_physical_num(ep_cfg);

	usb23_cmd(dev, USB23_DEPCMD(epn), USB23_DEPCMD_DEPCSTALL);
}

static void usb23_cmd_start_xfer(const struct device *dev,
		struct udc_ep_config *const ep_cfg)
{
	struct usb23_ep_data *ep_data = usb23_get_ep_data(dev, ep_cfg);
	uintptr_t trb_addr = (uintptr_t)ep_data->trb;
	int epn = usb23_get_ep_physical_num(ep_cfg);
	uint32_t idx;

	usb23_io_write(dev, USB23_DEPCMDPAR0(epn), HI32(trb_addr));
	usb23_io_write(dev, USB23_DEPCMDPAR1(epn), LO32(trb_addr));
	idx = usb23_cmd(dev, USB23_DEPCMD(epn), USB23_DEPCMD_DEPSTRTXFER);
	ep_data->xferrscidx = idx;
}

static void usb23_cmd_end_xfer(const struct device *dev,
		struct udc_ep_config *const ep_cfg)
{
	LOG_DBG("%s: EPx%02x", __func__, ep_cfg->addr);
	struct usb23_ep_data *ep_data = usb23_get_ep_data(dev, ep_cfg);
	int epn = usb23_get_ep_physical_num(ep_cfg);
	uint32_t reg;

	reg = ep_data->xferrscidx << USB23_DEPCMD_XFERRSCIDX_SHIFT;
	usb23_cmd(dev, USB23_DEPCMD(epn), USB23_DEPCMD_DEPENDXFER | reg);

	/* Mark the TRB as free for another request */
	usb23_set_trb(dev, ep_cfg, &(struct usb23_trb){0});
}

static void usb23_cmd_start_config(const struct device *dev,
		struct udc_ep_config *const ep_cfg)
{
	struct usb23_ep_data *ep_data = usb23_get_ep_data(dev, ep_cfg);
	int epn = usb23_get_ep_physical_num(ep_cfg);
	uint32_t reg;

	reg = ep_data->xferrscidx << USB23_DEPCMD_XFERRSCIDX_SHIFT;
	usb23_cmd(dev, USB23_DEPCMD(epn), USB23_DEPCMD_DEPSTARTCFG | reg);
}

/*------------------------------------------------------------------------------
--  Transfer Requests (TRB)
--------------------------------------------------------------------------------
--  USB23 receives transfer requests from this driver through a shared memory
--  buffer, resubmitted upon every new transfer (through either Start or Update
--  command).
*/

static void usb23_trb_single_buf(const struct device *dev,
		struct udc_ep_config *const ep_cfg,
		void *data, uint32_t status, uint32_t ctrl)
{
	struct usb23_trb trb = {
		.addr_lo = LO32((uintptr_t)data),
		.addr_hi = HI32((uintptr_t)data),
		.status = status,
		.ctrl = ctrl | USB23_TRB_CTRL_HWO,
	};

	usb23_set_trb(dev, ep_cfg, &trb);
	usb23_cmd_start_xfer(dev, ep_cfg);
}

static void usb23_trb_normal(const struct device *dev,
		struct udc_ep_config *const ep_cfg, void *data, size_t size)
{
	uint32_t ctrl = 0;

	LOG_DBG("\tEPx%02x NORMAL size=%d", ep_cfg->addr, size);
	ctrl |= USB23_TRB_CTRL_IOC;
	ctrl |= USB23_TRB_CTRL_LST;
	ctrl |= USB23_TRB_CTRL_TRBCTL_NORMAL;
	usb23_trb_single_buf(dev, ep_cfg, data, size, ctrl);
}

static void usb23_trb_ctrl_setup(const struct device *dev,
		struct udc_ep_config *const ep_cfg, void *data, size_t size)
{
	uint32_t ctrl = 0;

	LOG_DBG("\tEPx%02x TRB_CONTROL_SETUP size=%d", ep_cfg->addr, size);
	ctrl |= USB23_TRB_CTRL_IOC;
	ctrl |= USB23_TRB_CTRL_LST;
	ctrl |= USB23_TRB_CTRL_TRBCTL_CONTROL_SETUP;
	usb23_trb_single_buf(dev, ep_cfg, data, size, ctrl);
}

static void usb23_trb_ctrl_data(const struct device *dev,
		struct udc_ep_config *const ep_cfg, void *data, size_t size)
{
	uint32_t ctrl = 0;

	ctrl |= USB23_TRB_CTRL_IOC;
	ctrl |= USB23_TRB_CTRL_LST;
	if (ep_cfg->addr & USB_EP_DIR_IN) {
		LOG_DBG("\tEPx%02x TRB_CONTROL_DATA_1 size=%d", ep_cfg->addr, size);
		ctrl |= USB23_TRB_CTRL_TRBCTL_CONTROL_DATA_1;
	} else {
		// TODO This should not be required
		LOG_DBG("\tEPx%02x TRB_CONTROL_STATUS_3 size=%d", ep_cfg->addr, size);
		ctrl |= USB23_TRB_CTRL_TRBCTL_CONTROL_STATUS_3;
	}
	usb23_trb_single_buf(dev, ep_cfg, data, size, ctrl);
}

static void usb23_trb_ctrl_status(const struct device *dev,
		struct udc_ep_config *const ep_cfg, void *data, size_t size)
{
	uint32_t ctrl = 0;

	LOG_DBG("\tEPx%02x TRB_CONTROL_STATUS_2 size=%d", ep_cfg->addr, size);
	ctrl |= USB23_TRB_CTRL_IOC;
	ctrl |= USB23_TRB_CTRL_LST;
	ctrl |= USB23_TRB_CTRL_TRBCTL_CONTROL_STATUS_2;
	usb23_trb_single_buf(dev, ep_cfg, data, size, ctrl);
}

static bool usb23_trb_is_free(const struct device *dev,
		struct udc_ep_config *const ep_cfg)
{
	return usb23_get_trb(dev, ep_cfg).ctrl == 0x00000000;
}

static int usb23_trb_from_buf(const struct device *dev,
		struct udc_ep_config *const ep_cfg, struct net_buf *buf)
{
	struct udc_buf_info *bi = udc_get_buf_info(buf);
	struct usb23_ep_data *ep_data = usb23_get_ep_data(dev, ep_cfg);
	size_t n;

	if (!usb23_trb_is_free(dev, ep_cfg)) {
		return -ENOMEM;
	}

	/* Remember the current buffer */
	ep_data->buf = buf;

	n = ep_cfg->caps.in ? buf->len : buf->size;
	if (ep_cfg->caps.control) {
		if (bi->setup) {
			usb23_trb_ctrl_setup(dev, ep_cfg, buf->data, n);
		} else if (bi->data) {
			usb23_trb_ctrl_data(dev, ep_cfg, buf->data, n);
		} else if (bi->status) {
			usb23_trb_ctrl_status(dev, ep_cfg, buf->data, n);
		} else {
			__ASSERT_NO_MSG(false);
		}
	} else {
		usb23_trb_normal(dev, ep_cfg, buf->data, n);
	}
	return 0;
}

/*------------------------------------------------------------------------------
--  Transfers
------------------------------------------------------------------------------*/

/*
 * In usb23_ep_enqueue(), we only add the buffer to transfer to a list.
 * This is where we pick-up the elements from that list and place them
 * into the small TRB ring buffer.
 */
static void usb23_xfer_process_queue(const struct device *dev,
		struct udc_ep_config *const ep_cfg)
{
	struct net_buf *buf;

	if (ep_cfg->stat.halted) {
		return;
	}

	while (1) {
		buf = udc_buf_peek(dev, ep_cfg->addr);
		if (buf == NULL) {
			/* No more buffer in the queue */
			break;
		}

		if (usb23_trb_from_buf(dev, ep_cfg, buf) != 0) {
			/* Try again at the next round */
			LOG_DBG("\tEPx%02x !", ep_cfg->addr);
			break;
		}

		/* Pop from the list, now that it is submitted */
		udc_buf_get(dev, ep_cfg->addr);
	}
}

/*
 * Prepare a buffer for an incoming setup packet.
 */
static void usb23_xfer_ctrl_setup(const struct device *dev)
{
	struct net_buf *buf;

	buf = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT, 8);
	__ASSERT_NO_MSG(buf != NULL);
	udc_ep_buf_set_setup(buf);

	usb23_ep_enqueue(dev, udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT), buf);
}

static void usb23_xfer_ctrl_data(const struct device *dev,
		struct udc_ep_config *const ep_cfg, size_t size)
{
	struct net_buf *buf;
	struct udc_buf_info *bi;

	buf = udc_ctrl_alloc(dev, ep_cfg->addr, size);
	__ASSERT_NO_MSG(buf != NULL);
	bi = udc_get_buf_info(buf);
	bi->data = true;

	usb23_ep_enqueue(dev, ep_cfg, buf);
}

static void usb23_xfer_ctrl_data_out(const struct device *dev, size_t size)
{
	usb23_xfer_ctrl_data(dev, udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT), size);
}

static void usb23_xfer_ctrl_status(const struct device *dev,
		struct udc_ep_config *const ep_cfg, size_t size)
{
	struct net_buf *buf;
	struct udc_buf_info *bi;

	buf = udc_ctrl_alloc(dev, ep_cfg->addr, size);
	__ASSERT_NO_MSG(buf != NULL);
	bi = udc_get_buf_info(buf);
	bi->status = true;

	usb23_ep_enqueue(dev, ep_cfg, buf);
}

static void usb23_xfer_ctrl_status_in(const struct device *dev, size_t size)
{
	usb23_xfer_ctrl_status(dev, udc_get_ep_cfg(dev, USB_CONTROL_EP_IN), size);
}

static void usb23_xfer_ctrl_status_out(const struct device *dev, size_t size)
{
	usb23_xfer_ctrl_status(dev, udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT), size);
}

/*------------------------------------------------------------------------------
--  Events
--------------------------------------------------------------------------------
--  Process the events from the event ring buffer. Interrupts gives us a hint
--  that an event is available, which we fetch from a ring buffer shared with
--  the hardware.
*/

static void usb23_on_usb_reset(const struct device *dev)
{
	const struct usb23_config *config = dev->config;

	/* Reset all ongoing transfers on non-0 endpoints */
	for (int i = 1; i < config->num_of_eps; i++) {
		if (!usb23_trb_is_free(dev, &config->ep_cfg_in[i])) {
			usb23_cmd_end_xfer(dev, &config->ep_cfg_in[i]);
		}
		usb23_cmd_clear_stall(dev, &config->ep_cfg_in[i]);

		if (!usb23_trb_is_free(dev, &config->ep_cfg_out[i])) {
			usb23_cmd_end_xfer(dev, &config->ep_cfg_out[i]);
		}
		usb23_cmd_clear_stall(dev, &config->ep_cfg_out[i]);
	}

	/* Perform the USB reset operations manually to improve latency */
	usb23_set_address(dev, 0);

	/* Let Zephyr set the device address 0 */
	udc_submit_event(dev, UDC_EVT_RESET, 0);
}

static void usb23_on_connect_done(const struct device *dev)
{
	struct udc_ep_config *ep_cfg;
	int mps = 0;

	LOG_DBG("%s", __func__);

	/* Adjust parameters against the connection speed */
	switch (usb23_io_read(dev, USB23_DSTS) & USB23_DSTS_CONNECTSPD_MASK) {
	CASE(USB23_DSTS_CONNECTSPD_FS);
	CASE(USB23_DSTS_CONNECTSPD_HS);
		mps = 64;
		usb23_io_set(dev, USB23_GUSB3PIPECTL, USB23_GUSB3PIPECTL_SUSPENDENABLE);
		break;
	CASE(USB23_DSTS_CONNECTSPD_SS);
		mps = 512;
		usb23_io_set(dev, USB23_GUSB2PHYCFG, USB23_GUSB2PHYCFG_SUSPHY);
		break;
	}
	__ASSERT_NO_MSG(mps != 0);

	/* Letting GCTL unchanged */

	/* Reconfigure EPx00 connection speed */
	ep_cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);
	ep_cfg->mps = mps;
	usb23_cmd_ep_config(dev, ep_cfg);

	/* Reconfigure EPx80 connection speed */
	ep_cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_IN);
	ep_cfg->mps = mps;
	usb23_cmd_ep_config(dev, ep_cfg);

	/* Letting GTXFIFOSIZn unchanged */

	/* Check the pending buffers again */
	usb23_xfer_process_queue(dev, ep_cfg);
}

static void usb23_on_link_state_event(const struct device *dev)
{
	uint32_t reg;

	reg = usb23_io_read(dev, USB23_DSTS);

	switch (reg & USB23_DSTS_CONNECTSPD_MASK) {
	case USB23_DSTS_CONNECTSPD_SS:
		switch (reg & USB23_DSTS_USBLNKST_MASK) {
		CASE(USB23_DSTS_USBLNKST_USB3_U0);
			break;
		CASE(USB23_DSTS_USBLNKST_USB3_U1);
			break;
		CASE(USB23_DSTS_USBLNKST_USB3_U2);
			break;
		CASE(USB23_DSTS_USBLNKST_USB3_U3);
			break;
		CASE(USB23_DSTS_USBLNKST_USB3_SS_DIS);
			break;
		CASE(USB23_DSTS_USBLNKST_USB3_RX_DET);
			break;
		CASE(USB23_DSTS_USBLNKST_USB3_SS_INACT);
			break;
		CASE(USB23_DSTS_USBLNKST_USB3_POLL);
			break;
		CASE(USB23_DSTS_USBLNKST_USB3_RECOV);
			break;
		CASE(USB23_DSTS_USBLNKST_USB3_HRESET);
			break;
		CASE(USB23_DSTS_USBLNKST_USB3_CMPLY);
			break;
		CASE(USB23_DSTS_USBLNKST_USB3_LPBK);
			break;
		CASE(USB23_DSTS_USBLNKST_USB3_RESET_RESUME);
			break;
		default:
			LOG_ERR("unknown USB3 link state");
		}
		break;
	case USB23_DSTS_CONNECTSPD_HS:
	case USB23_DSTS_CONNECTSPD_FS:
		switch (reg & USB23_DSTS_USBLNKST_MASK) {
		CASE(USB23_DSTS_USBLNKST_USB2_ON_STATE);
			break;
		CASE(USB23_DSTS_USBLNKST_USB2_SLEEP_STATE);
			break;
		CASE(USB23_DSTS_USBLNKST_USB2_SUSPEND_STATE);
			break;
		CASE(USB23_DSTS_USBLNKST_USB2_DISCONNECTED);
			break;
		CASE(USB23_DSTS_USBLNKST_USB2_EARLY_SUSPEND);
			break;
		CASE(USB23_DSTS_USBLNKST_USB2_RESET);
			break;
		CASE(USB23_DSTS_USBLNKST_USB2_RESUME);
			break;
		default:
			LOG_ERR("unknown USB2 link state");
		}
		break;
	default:
		LOG_ERR("unknown connection speed");
	}
}

static void usb23_on_device_event(const struct device *dev,
		struct usb23_devt devt)
{
	switch (devt.type) {
	CASE(USB23_DEVT_TYPE_DISCONNEVT);
		break;
	CASE(USB23_DEVT_TYPE_USBRST);
		usb23_on_usb_reset(dev);
		break;
	CASE(USB23_DEVT_TYPE_CONNECTDONE);
		usb23_on_connect_done(dev);
		break;
	CASE(USB23_DEVT_TYPE_ULSTCHNG);
		usb23_on_link_state_event(dev);
		break;
	CASE(USB23_DEVT_TYPE_WKUPEVT);
		break;
	CASE(USB23_DEVT_TYPE_SUSPEND);
		break;
	CASE(USB23_DEVT_TYPE_SOF);
		break;
	CASE(USB23_DEVT_TYPE_ERRTICERR);
		break;
	CASE(USB23_DEVT_TYPE_CMDCMPLT);
		break;
	CASE(USB23_DEVT_TYPE_EVNTOVERFLOW);
		break;
	CASE(USB23_DEVT_TYPE_VNDRDEVTSTRCVED);
		break;
	default:
		LOG_ERR("unknown device event: 0x%08d", *(uint32_t *)&devt);
	}
}

/*-- Control-Write --*/

/* OUT (setup) */
static void usb23_on_ctrl_write_setup(const struct device *dev,
		struct udc_ep_config *const ep_cfg, struct net_buf *buf)
{
	LOG_DBG("%s", __func__);
	usb23_xfer_ctrl_data_out(dev, udc_data_stage_length(buf));
}

/* OUT (data) */
static void usb23_on_ctrl_write_data(const struct device *dev,
		struct udc_ep_config *const ep_cfg, struct net_buf *buf)
{
	LOG_DBG("%s", __func__);
	udc_ctrl_update_stage(dev, buf);
	udc_ctrl_submit_s_out_status(dev, buf);
}

/* IN (status) */
static void usb23_on_ctrl_write_status(const struct device *dev,
		struct udc_ep_config *const ep_cfg, struct net_buf *buf)
{
	LOG_DBG("%s", __func__);
	udc_ctrl_submit_status(dev, buf);
	udc_ctrl_update_stage(dev, buf);
}

/*-- Control Read --*/

/* OUT (setup) */
static void usb23_on_ctrl_read_setup(const struct device *dev,
		struct udc_ep_config *const ep_cfg, struct net_buf *buf)
{
	LOG_DBG("%s", __func__);
	usb23_xfer_ctrl_status_out(dev, 0);
	udc_ctrl_submit_s_in_status(dev);
}

/* IN (data) */
static void usb23_on_ctrl_read_data(const struct device *dev,
		struct udc_ep_config *const ep_cfg, struct net_buf *buf)
{
	LOG_DBG("%s", __func__);
	udc_ctrl_update_stage(dev, buf);
}

/* OUT (status) */
static void usb23_on_ctrl_read_status(const struct device *dev,
		struct udc_ep_config *const ep_cfg, struct net_buf *buf)
{
	LOG_DBG("%s", __func__);
	udc_ctrl_submit_status(dev, buf);
	udc_ctrl_update_stage(dev, buf);
}

/*-- No-Data Control --*/

/* OUT (setup) */
static void usb23_on_ctrl_nodata_setup(const struct device *dev,
		struct udc_ep_config *const ep_cfg, struct net_buf *buf)
{
	LOG_DBG("%s", __func__);
	udc_ctrl_submit_s_status(dev);
}

/* IN (status) */
static void usb23_on_ctrl_nodata_status(const struct device *dev,
		struct udc_ep_config *const ep_cfg, struct net_buf *buf)
{
	LOG_DBG("%s", __func__);
	udc_ctrl_submit_status(dev, buf);
	udc_ctrl_update_stage(dev, buf);
}

/*--*/

static void usb23_on_ctrl_setup(const struct device *dev,
		struct udc_ep_config *const ep_cfg, struct net_buf *buf)
{
	/* Move to the next state to be able to differentiate in/out/no-data */
	udc_ctrl_update_stage(dev, buf);

	if (udc_ctrl_stage_is_data_out(dev)) {
		usb23_on_ctrl_write_setup(dev, ep_cfg, buf);
	} else if (udc_ctrl_stage_is_data_in(dev)) {
		usb23_on_ctrl_read_setup(dev, ep_cfg, buf);
	} else if (udc_ctrl_stage_is_no_data(dev)) {
		usb23_on_ctrl_nodata_setup(dev, ep_cfg, buf);
	} else {
		LOG_ERR("unknown setup stage");
	}
}

static void usb23_on_ctrl_data(const struct device *dev,
		struct udc_ep_config *const ep_cfg, struct net_buf *buf)
{
	if (udc_ctrl_stage_is_data_out(dev)) {
		usb23_on_ctrl_write_data(dev, ep_cfg, buf);
	} else if (udc_ctrl_stage_is_data_in(dev)) {
		usb23_on_ctrl_read_data(dev, ep_cfg, buf);
	} else {
		LOG_ERR("unknown data stage");
	}
}

static void usb23_on_ctrl_status(const struct device *dev,
		struct udc_ep_config *const ep_cfg, struct net_buf *buf)
{
	if (udc_ctrl_stage_is_status_in(dev)) {
		usb23_on_ctrl_write_status(dev, ep_cfg, buf);
	} else if (udc_ctrl_stage_is_status_out(dev)) {
		usb23_on_ctrl_read_status(dev, ep_cfg, buf);
	} else if (udc_ctrl_stage_is_no_data(dev)) {
		usb23_on_ctrl_nodata_status(dev, ep_cfg, buf);
	} else {
		LOG_ERR("unknown status stage");
		return;
	}

	/* Prepare the next setup transfer */
	usb23_xfer_ctrl_setup(dev);
}

static void usb23_on_xfer_complete(const struct device *dev,
		struct udc_ep_config *const ep_cfg)
{
	struct usb23_ep_data *ep_data = usb23_get_ep_data(dev, ep_cfg);
	struct usb23_trb trb = usb23_get_trb(dev, ep_cfg);
	struct net_buf *buf = ep_data->buf;
	int err;

	/* Latency optimization: set the address immediately to be able to
	 * be able to ACK/NAK the first packets from the host with the new
	 * address, otherwise the host issue a reset */
	if (buf->size > 2 && buf->data[0] == 0 && buf->data[1] == 5) {
		usb23_set_address(dev, buf->data[2]);
	}

	buf->len = buf->size - GETFIELD(trb.status, USB23_TRB_STATUS_BUFSIZ);
	LOG_DBG("buf->len=%d", buf->len);

	__ASSERT_NO_MSG((uintptr_t)buf->data == U64(trb.addr_hi, trb.addr_lo));
	__ASSERT_NO_MSG(trb.ctrl != 0x00000000);
	__ASSERT_NO_MSG((trb.ctrl & USB23_TRB_CTRL_HWO) == 0);
	__ASSERT_NO_MSG((trb.status & USB23_TRB_STATUS_TRBSTS_MASK) ==
			USB23_TRB_STATUS_TRBSTS_OK);
	LOG_HEXDUMP_DBG(buf->data, buf->len, "BUFFER");

	switch (trb.ctrl & USB23_TRB_CTRL_TRBCTL_MASK) {
	CASE(USB23_TRB_CTRL_TRBCTL_CONTROL_SETUP);
		usb23_on_ctrl_setup(dev, ep_cfg, buf);
		break;
	CASE(USB23_TRB_CTRL_TRBCTL_CONTROL_DATA_1);
		usb23_on_ctrl_data(dev, ep_cfg, buf);
		break;
	CASE(USB23_TRB_CTRL_TRBCTL_CONTROL_STATUS_3);
		usb23_on_ctrl_data(dev, ep_cfg, buf);
		break;
	CASE(USB23_TRB_CTRL_TRBCTL_CONTROL_STATUS_2);
		usb23_on_ctrl_status(dev, ep_cfg, buf);
		break;
	default:
		err = udc_submit_ep_event(dev, buf, 0);
		__ASSERT_NO_MSG(err == 0);
		break;
	}

	/* Mark the TRB as free for another request */
	usb23_set_trb(dev, ep_cfg, &(struct usb23_trb){0});

	/* Check the pending buffers again */
	usb23_xfer_process_queue(dev, ep_cfg);
}

static void usb23_on_endpoint_event(const struct device *dev,
		struct usb23_depevt depevt)
{
	int epn = depevt.ep_num;
	struct udc_ep_config *const ep_cfg = usb23_get_ep_cfg(dev, epn);

	switch (depevt.type) {
	CASE(USB23_DEPEVT_TYPE_XFERCOMPLETE, "EPx%02x", ep_cfg->addr);
		__ASSERT_NO_MSG((depevt.status & USB23_DEPEVT_STATUS_BUSERR) == 0);
		usb23_on_xfer_complete(dev, ep_cfg);
		break;
	CASE(USB23_DEPEVT_TYPE_XFERINPROGRESS, "EPx%02x", ep_cfg->addr);
		break;
	CASE(USB23_DEPEVT_TYPE_XFERNOTREADY, "EPx%02x", ep_cfg->addr);
		switch (depevt.status & USB23_DEPEVT_STATUS_B3_MASK) {
		CASE(USB23_DEPEVT_STATUS_B3_CONTROL_SETUP);
			break;
		CASE(USB23_DEPEVT_STATUS_B3_CONTROL_DATA);
			break;
		CASE(USB23_DEPEVT_STATUS_B3_CONTROL_STATUS);
			break;
		}

		/* Check the pending buffers again */
		usb23_xfer_process_queue(dev, ep_cfg);
		break;
	CASE(USB23_DEPEVT_TYPE_RXTXFIFOEVT, "EPx%02x", ep_cfg->addr);
		break;
	CASE(USB23_DEPEVT_TYPE_STREAMEVT, "EPx%02x", ep_cfg->addr);
		break;
	CASE(USB23_DEPEVT_TYPE_EPCMDCMPLT, "EPx%02x", ep_cfg->addr);
		break;
	default:
		LOG_ERR("unknown endpoint event: 0x%08d", *(uint32_t *)&depevt);
	}
}

static void usb23_on_event(struct k_work *work)
{
	const struct usb23_data *priv =
			CONTAINER_OF(work, struct usb23_data, work);
	const struct device *dev = priv->dev;

	/* Process each pending event from the list */
	while (usb23_io_read(dev, USB23_GEVNTCOUNT(0)) > 0) {
		union usb23_evt evt = usb23_get_next_evt(dev);

		LOG_DBG("");

		/* We can already release the resource now that we copied it */
		usb23_io_write(dev, USB23_GEVNTCOUNT(0), sizeof(evt));

		if (evt.raw == 0x00000000) {
			LOG_ERR("empty event received");
		} else if (evt.devt.category) {
			usb23_on_device_event(dev, evt.devt);
		} else {
			usb23_on_endpoint_event(dev, evt.depevt);
		}
	}
}

void usb23_irq_handler(void *ptr)
{
	const struct device *dev = ptr;
	const struct usb23_config *config = dev->config;
	struct usb23_data *priv = udc_get_private(dev);

	config->irq_clear_func();
	usb23_on_event(&priv->work);
}

/*------------------------------------------------------------------------------
--  UDC API
--------------------------------------------------------------------------------
--  Interface called by Zehpyr from the upper levels of abstractions.
*/

static int usb23_ep_enqueue(const struct device *dev,
		struct udc_ep_config *const ep_cfg, struct net_buf *buf)
{
	LOG_DBG("QUEUE EPx%02x", ep_cfg->addr);
	udc_buf_put(ep_cfg, buf);
	usb23_xfer_process_queue(dev, ep_cfg);
	return 0;
}

static int usb23_ep_dequeue(const struct device *dev,
		struct udc_ep_config *const ep_cfg)
{
	unsigned int lock_key;
	struct net_buf *buf;

	LOG_DBG("%s", __func__);

	lock_key = irq_lock();

	buf = udc_buf_get_all(dev, ep_cfg->addr);
	if (buf) {
		udc_submit_ep_event(dev, buf, -ECONNABORTED);
	}

	irq_unlock(lock_key);
	return 0;
}

static int usb23_ep_disable(const struct device *dev,
		struct udc_ep_config *const ep_cfg)
{
	int epn = usb23_get_ep_physical_num(ep_cfg);

	LOG_DBG("%s EPx%02x", __func__, ep_cfg->addr);
	usb23_io_clear(dev, USB23_DALEPENA, BIT(epn));
	return 0;
}

/*
 * Halt endpoint. Halted endpoint should respond with a STALL handshake.
 */
static int usb23_ep_set_halt(const struct device *dev,
		struct udc_ep_config *ep_cfg)
{
	LOG_WRN("Setting stall state on endpoint EPx%02x", ep_cfg->addr);

	if (ep_cfg->addr == USB_CONTROL_EP_IN) {
		/* For the control endpoint, only OUT is stalled */
		ep_cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);
	}
	usb23_cmd_set_stall(dev, ep_cfg);

	ep_cfg->stat.halted = true;
	return 0;
}

static int usb23_ep_clear_halt(const struct device *dev,
		struct udc_ep_config *const ep_cfg)
{
	LOG_WRN("Clearing stall state on endpoint EPx%02x", ep_cfg->addr);
	__ASSERT_NO_MSG(ep_cfg->addr != USB_CONTROL_EP_OUT);
	__ASSERT_NO_MSG(ep_cfg->addr != USB_CONTROL_EP_IN);

	usb23_cmd_clear_stall(dev, ep_cfg);
	ep_cfg->stat.halted = false;
	return 0;
}

static int usb23_check_address(const struct device *dev, const uint8_t addr)
{
	/* The address is set in the code earlier to improve latency, only
	 * checking that it is still the value done for consistency. */
	__ASSERT_NO_MSG(GETFIELD(usb23_io_read(dev, USB23_DCFG),
			USB23_DCFG_DEVADDR) == addr);
	return 0;
}

static int usb23_set_address(const struct device *dev, const uint8_t addr)
{
	LOG_INF("ADDR %d", addr);

	/* Configure the new address */
	usb23_io_field(dev, USB23_DCFG, USB23_DCFG_DEVADDR_MASK,
			addr << USB23_DCFG_DEVADDR_SHIFT);

	/* Re-apply the same endpoint configuration */
	usb23_cmd_ep_config(dev, udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT));
	usb23_cmd_ep_config(dev, udc_get_ep_cfg(dev, USB_CONTROL_EP_IN));

	return 0;
}

static int usb23_host_wakeup(const struct device *dev)
{
	return 0;
}

static enum udc_bus_speed usb23_device_speed(const struct device *dev)
{
	switch (usb23_io_read(dev, USB23_DSTS) & USB23_DSTS_CONNECTSPD_MASK) {
	case USB23_DSTS_CONNECTSPD_HS:
		return UDC_BUS_SPEED_HS;
	case USB23_DSTS_CONNECTSPD_FS:
		return UDC_BUS_SPEED_FS;
	case USB23_DSTS_CONNECTSPD_SS:
		return UDC_BUS_SPEED_SS;
	}
	__ASSERT(0, "unknown device speed");
	return 0;
}

static int usb23_enable(const struct device *dev)
{
	const struct usb23_config *config = dev->config;

	LOG_DBG("%s", __func__);

	/* Bootstrap: prepare reception of the initial Setup packet */
	usb23_xfer_ctrl_setup(dev);

	usb23_io_set(dev, USB23_DCTL, USB23_DCTL_RUNSTOP);
	config->irq_enable_func();
	return 0;
}

static int usb23_disable(const struct device *dev)
{
	LOG_DBG("%s", __func__);
	return 0;
}

/*
 * Shut down the controller completely
 */
static int usb23_shutdown(const struct device *dev)
{
	LOG_DBG("%s", __func__);
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

static int usb23_lock(const struct device *dev)
{
	return udc_lock_internal(dev, K_FOREVER);
}

static int usb23_unlock(const struct device *dev)
{
	return udc_unlock_internal(dev);
}

/*------------------------------------------------------------------------------
--  Hardware Init
--------------------------------------------------------------------------------
--  Prepare the driver and the hardware to being used.
--  This goes through register configuration and register commands.
*/

static int usb23_ep_enable(const struct device *dev,
		struct udc_ep_config *const ep_cfg)
{
	int epn = usb23_get_ep_physical_num(ep_cfg);

	LOG_DBG("%s EPx%02x", __func__, ep_cfg->addr);

	/* Issue configuration commands for this endpoint */
	usb23_cmd_ep_config(dev, ep_cfg);
	usb23_cmd_ep_xfer_config(dev, ep_cfg);

	/* Starting from here, the endpoint can be used */
	usb23_io_set(dev, USB23_DALEPENA, USB23_DALEPENA_USBACTEP(epn));

	return 0;
}

/*
 * Prepare and configure most of the parts, if the controller has a way
 * of detecting VBUS activity it should be enabled here.
 * Only usb23_enable() makes device visible to the host.
 */
static int usb23_init(const struct device *dev)
{
	struct usb23_data *priv = udc_get_private(dev);
	struct udc_data *data = dev->data;
	struct udc_ep_config *ep_cfg;
	uint32_t reg, core, rel;
	int err = 0;

	/* Read the chip identification */
	reg = usb23_io_read(dev, USB23_GCOREID);
	core = GETFIELD(reg, USB23_GCOREID_CORE);
	rel = GETFIELD(reg, USB23_GCOREID_REL);
	LOG_INF("coreid=0x%04x release=0x%04x", core, rel);
	__ASSERT_NO_MSG(core == 0x5533);

	/* Issue a soft reset to the core and USB2 and USB3 PHY */
	usb23_io_set(dev, USB23_GCTL, USB23_GCTL_CORESOFTRESET);
	usb23_io_set(dev, USB23_GUSB3PIPECTL, USB23_GUSB3PIPECTL_PHYSOFTRST);
	usb23_io_set(dev, USB23_GUSB2PHYCFG, USB23_GUSB2PHYCFG_PHYSOFTRST);
	k_sleep(K_MSEC(1)); // TODO: reduce amount of wait time

	/* Teriminate the reset of the USB2 and USB3 PHY first */
	k_sleep(K_MSEC(1)); // TODO: reduce amount of wait time
	usb23_io_clear(dev, USB23_GUSB3PIPECTL, USB23_GUSB3PIPECTL_PHYSOFTRST);
	usb23_io_clear(dev, USB23_GUSB2PHYCFG, USB23_GUSB2PHYCFG_PHYSOFTRST);

	/* Teriminate the reset of the USB23 core after it */
	usb23_io_clear(dev, USB23_GCTL, USB23_GCTL_CORESOFTRESET);

	/* Initialize USB2 PHY Lattice wrappers */
	usb23_io_set(dev, USB23_U2PHYCTRL1, USB23_U2PHYCTRL1_SEL_INTERNALCLK);
	usb23_io_set(dev, USB23_U2PHYCTRL2, USB23_U2PHYCTRL2_REFCLK_SEL);

	/* Initialize USB3 PHY Lattice wrappers */
	usb23_io_set(dev, USB23_U3PHYCTRL1, BIT(22));
	usb23_io_clear(dev, USB23_U3PHYCTRL4, USB23_U3PHYCTRL4_INT_CLOCK);

	/* Configure and reset the Device Controller */
	usb23_io_write(dev, USB23_DCTL, 0x40f00000); // TODO decode
	usb23_io_wait_go_low(dev, USB23_DCTL, USB23_DCTL_CSFTRST);

	/* Letting GSBUSCFG0 and GSBUSCFG1 unchanged */
	/* Letting GTXTHRCFG and GRXTHRCFG unchanged */

	/* Issue a soft reset to USB2 PHY */
	usb23_io_set(dev, USB23_GUSB2PHYCFG, USB23_GUSB2PHYCFG_PHYSOFTRST);
	usb23_io_clear(dev, USB23_GUSB2PHYCFG, USB23_GUSB2PHYCFG_PHYSOFTRST);

	/* Setting fifo size for both TX and RX */
	usb23_io_write(dev, USB23_GTXFIFOSIZ(0), 512);
	usb23_io_write(dev, USB23_GRXFIFOSIZ(0), 512);

	/* Setup the event buffer address, size and start event reception */
	for (int i = 0; i < ARRAY_SIZE(priv->evt_buf); i++) {
		priv->evt_buf[i] = (union usb23_evt){0};
	}
	flush_l2_cache();
	flush_cpu_dcache();
	usb23_io_write(dev, USB23_GEVNTADR_LO(0), LO32((uintptr_t)priv->evt_buf));
	usb23_io_write(dev, USB23_GEVNTADR_HI(0), HI32((uintptr_t)priv->evt_buf));
	usb23_io_write(dev, USB23_GEVNTSIZ(0), sizeof(priv->evt_buf));
	usb23_io_write(dev, USB23_GEVNTCOUNT(0), 0);

	/* Set the USB device configuration, including max supported speed */
	usb23_io_write(dev, USB23_DCFG, USB23_DCFG_PERFRINT_90);
	if (data->caps.ss) {
		usb23_io_set(dev, USB23_DCFG, USB23_DCFG_DEVSPD_SUPER_SPEED);
	} else if (data->caps.hs) {
		usb23_io_set(dev, USB23_DCFG, USB23_DCFG_DEVSPD_HIGH_SPEED);
	} else {
		usb23_io_set(dev, USB23_DCFG, USB23_DCFG_DEVSPD_FULL_SPEED);
	}

	/* Enable reception of USB events */
	usb23_io_write(dev, USB23_DEVTEN, 0
		| USB23_DEVTEN_INACTTIMEOUTRCVEDEN
		| USB23_DEVTEN_VNDRDEVTSTRCVEDEN
		| USB23_DEVTEN_EVNTOVERFLOWEN
		| USB23_DEVTEN_CMDCMPLTEN
		| USB23_DEVTEN_ERRTICERREN
		| USB23_DEVTEN_HIBERNATIONREQEVTEN
		| USB23_DEVTEN_WKUPEVTEN
	//	| USB23_DEVTEN_ULSTCNGEN
		| USB23_DEVTEN_CONNECTDONEEN
		| USB23_DEVTEN_USBRSTEN
		| USB23_DEVTEN_DISCONNEVTEN
	);

	/* Only to be done for endpoint 0 and 1 at this stage */
	usb23_cmd_start_config(dev, udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT));
	usb23_cmd_start_config(dev, udc_get_ep_cfg(dev, USB_CONTROL_EP_IN));

	/* Configure the control OUT endpoint */
	ep_cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);
	err = udc_ep_enable_internal(dev, USB_CONTROL_EP_OUT, USB_EP_TYPE_CONTROL, ep_cfg->mps, 0);
	__ASSERT_NO_MSG(err == 0);

	/* Configure the control IN endpoint */
	ep_cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);
	err = udc_ep_enable_internal(dev, USB_CONTROL_EP_IN, USB_EP_TYPE_CONTROL,
		ep_cfg->mps, 0);
	__ASSERT_NO_MSG(err == 0);

	return err;
}

/*------------------------------------------------------------------------------
--  Software Init
--------------------------------------------------------------------------------
--  Setup the default values and configure software variables and structs.
--  No effect on hardware.
*/

static int usb23_ep_preinit(const struct device *dev,
		struct udc_ep_config *const ep_cfg, int addr, int mps)
{
	int err;

	/* Generic properties for Zephyr */
	ep_cfg->addr = addr;
	if (ep_cfg->addr & USB_EP_DIR_IN) {
		ep_cfg->caps.in = 1;
	} else {
		ep_cfg->caps.out = 1;
	}
	if (ep_cfg->addr == 0x00 || ep_cfg->addr == 0x80) {
		ep_cfg->caps.control = 1;
	} else {
		ep_cfg->caps.bulk = 1;
		ep_cfg->caps.interrupt = 1;
		ep_cfg->caps.iso = 1;
	}
	ep_cfg->caps.mps = mps;

	err = udc_register_ep(dev, ep_cfg);
	if (err != 0) {
		LOG_ERR("Failed to register endpoint");
		return err;
	}

	/* Mark the active TRB as vacant */
	usb23_set_trb(dev, ep_cfg, &(struct usb23_trb){0});

	return 0;
}

/*
 * Initialize the controller and endpoints capabilities,
 * register endpoint structures, no hardware I/O yet.
 */
static int usb23_driver_preinit(const struct device *dev)
{
	const struct usb23_config *config = dev->config;
	struct udc_data *data = dev->data;
	struct usb23_data *priv = udc_get_private(dev);
	struct udc_ep_config *const ep_cfg_in = config->ep_cfg_in;
	struct udc_ep_config *const ep_cfg_out = config->ep_cfg_out;
	uint16_t mps = 1023;

	k_mutex_init(&data->mutex);
	k_work_init(&priv->work, &usb23_on_event);

	data->caps.rwup = true;
	switch (config->speed_idx) {
	case 3:
		data->caps.mps0 = UDC_MPS0_512;
		data->caps.ss = true;
		data->caps.hs = true;
		mps = 1024;
		break;
	case 2:
		data->caps.mps0 = UDC_MPS0_64;
		data->caps.hs = true;
		mps = 1024;
		break;
	case 1:
		data->caps.mps0 = UDC_MPS0_64; // TODO: adjust
		mps = 1024; // TODO: adjust
		break;
	default:
		LOG_ERR("Not implemented");
	}

	for (int i = 0; i < config->num_of_eps; i++) {
		usb23_ep_preinit(dev, &ep_cfg_out[i], i | USB_EP_DIR_OUT, mps);
		usb23_ep_preinit(dev, &ep_cfg_in[i], i | USB_EP_DIR_IN, mps);
	}

	return 0;
}

static const struct udc_api usb23_api = {
	.lock = usb23_lock,
	.unlock = usb23_unlock,
	.device_speed = usb23_device_speed,
	.init = usb23_init,
	.enable = usb23_enable,
	.disable = usb23_disable,
	.shutdown = usb23_shutdown,
	.set_address = usb23_check_address,
	.host_wakeup = usb23_host_wakeup,
	.ep_enable = usb23_ep_enable,
	.ep_disable = usb23_ep_disable,
	.ep_set_halt = usb23_ep_set_halt,
	.ep_clear_halt = usb23_ep_clear_halt,
	.ep_enqueue = usb23_ep_enqueue,
	.ep_dequeue = usb23_ep_dequeue,
};

#define USB23_DEVICE_DEFINE(n)							\
	static void usb23_irq_enable(void)					\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),		\
				usb23_irq_handler, DEVICE_DT_INST_GET(n), 0);	\
		irq_enable(DT_INST_IRQN(n));					\
		*(volatile uint32_t *)DT_INST_REG_ADDR_BY_NAME(n, ev_enable) =	\
				0xffffffff;					\
	}									\
										\
	static void usb23_irq_clear(void)					\
	{									\
		*(volatile uint32_t *)DT_INST_REG_ADDR_BY_NAME(n, ev_pending) =	\
				0xffffffff;					\
	}									\
										\
	static struct udc_ep_config						\
		ep_cfg_out[DT_INST_PROP(n, num_bidir_endpoints)];		\
	static struct udc_ep_config						\
		ep_cfg_in[DT_INST_PROP(n, num_bidir_endpoints)];		\
	static const struct usb23_config usb23_config_##n = {			\
		.base = DT_INST_REG_ADDR_BY_NAME(n, base),			\
		.num_of_eps = DT_INST_PROP(n, num_bidir_endpoints),		\
		.ep_cfg_out = ep_cfg_out,					\
		.ep_cfg_in = ep_cfg_in,						\
		.speed_idx = DT_ENUM_IDX(DT_DRV_INST(n), maximum_speed),	\
		.irq_enable_func = usb23_irq_enable,				\
		.irq_clear_func = usb23_irq_clear,				\
	};									\
										\
	static struct usb23_ep_data						\
		ep_data[DT_INST_PROP(n, num_bidir_endpoints) * 2];		\
	static struct usb23_data udc_priv_##n = {				\
		.dev = DEVICE_DT_INST_GET(n),					\
		.ep_data = ep_data,						\
	};									\
										\
	static struct udc_data udc_data_##n = {					\
		.mutex = Z_MUTEX_INITIALIZER(udc_data_##n.mutex),		\
		.priv = &udc_priv_##n,						\
	};									\
										\
	DEVICE_DT_INST_DEFINE(n, usb23_driver_preinit, NULL,			\
			&udc_data_##n, &usb23_config_##n,			\
			POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	\
			&usb23_api);

DT_INST_FOREACH_STATUS_OKAY(USB23_DEVICE_DEFINE)
