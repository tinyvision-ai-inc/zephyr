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

static const struct udc_api usb23_api = {
	.lock = usb23_api_lock,
	.unlock = usb23_api_unlock,
	.device_speed = usb23_api_device_speed,
	.init = usb23_api_init,
	.enable = usb23_api_enable,
	.disable = usb23_api_disable,
	.shutdown = usb23_api_shutdown,
	.set_address = usb23_api_set_address,
	.set_exit_latency = usb23_api_set_exit_latency,
	.host_wakeup = usb23_api_host_wakeup,
	.ep_enable = usb23_api_ep_enable,
	.ep_disable = usb23_api_ep_disable,
	.ep_set_halt = usb23_api_ep_set_halt,
	.ep_clear_halt = usb23_api_ep_clear_halt,
	.ep_enqueue = usb23_api_ep_enqueue,
	.ep_dequeue = usb23_api_ep_dequeue,
};

#define NUM_OF_EPS(n)          (DT_INST_PROP(n, num_bidir_endpoints) * 2)
#define NUM_OF_TRBS(inst, epn) DT_PROP_BY_IDX(inst, num_endpoint_trb, epn)

#define USB23_EP_TRB_BUF_DEFINE(n, m, epn)                                                         \
	static struct usb23_trb usb23_dma_trb_buf_##n##_##epn[NUM_OF_TRBS(n, epn)];

#define USB23_EP_NET_BUF_DEFINE(n, m, epn)                                                         \
	static struct net_buf *usb23_net_buf_##n##_##epn[NUM_OF_TRBS(n, epn)];

#define USB23_EP_DATA_ENTRY(n, m, epn)                                                             \
	{                                                                                          \
		.num_of_trbs = NUM_OF_TRBS(n, epn),                                                \
		.trb_buf = usb23_dma_trb_buf_##n##_##epn,                                          \
		.net_buf = usb23_net_buf_##n##_##epn,                                              \
	},

#define USB23_DEVICE_DEFINE(n)                                                                     \
                                                                                                   \
	static void usb23_irq_enable(void)                                                         \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), usb23_irq_handler,          \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
		*(volatile uint32_t *)DT_INST_REG_ADDR_BY_NAME(n, ev_enable) = 0xffffffff;         \
	}                                                                                          \
                                                                                                   \
	static void usb23_irq_clear(void)                                                          \
	{                                                                                          \
		*(volatile uint32_t *)DT_INST_REG_ADDR_BY_NAME(n, ev_pending) = 0xffffffff;        \
	}                                                                                          \
                                                                                                   \
	union usb23_evt usb23_dma_evt_buf_##n[CONFIG_USB23_EVT_NUM];                               \
	struct udc_ep_config usb23_ep_cfg_##n[NUM_OF_EPS(n)];                                      \
                                                                                                   \
	DT_INST_FOREACH_PROP_ELEM(n, num_endpoint_trb, USB23_EP_TRB_BUF_DEFINE);                   \
	DT_INST_FOREACH_PROP_ELEM(n, num_endpoint_trb, USB23_EP_NET_BUF_DEFINE);                   \
                                                                                                   \
	static struct usb23_ep_data usb23_ep_data_##n[NUM_OF_EPS(n)] = {                           \
		DT_INST_FOREACH_PROP_ELEM(n, num_endpoint_trb, USB23_EP_DATA_ENTRY)};              \
                                                                                                   \
	static const struct usb23_config usb23_config_##n = {                                      \
		.base = DT_INST_REG_ADDR_BY_NAME(n, base),                                         \
		.discard = DT_INST_REG_ADDR_BY_NAME(n, discard),                                   \
		.num_of_eps = NUM_OF_EPS(n),                                                       \
		.ep_cfg = usb23_ep_cfg_##n,                                                        \
		.ep_data = usb23_ep_data_##n,                                                      \
		.speed_idx = DT_ENUM_IDX(DT_DRV_INST(n), maximum_speed),                           \
		.irq_enable_func = usb23_irq_enable,                                               \
		.irq_clear_func = usb23_irq_clear,                                                 \
		.evt_buf = usb23_dma_evt_buf_##n,                                                  \
	};                                                                                         \
                                                                                                   \
	static struct usb23_data udc_priv_##n = {                                                  \
		.dev = DEVICE_DT_INST_GET(n),                                                      \
	};                                                                                         \
                                                                                                   \
	static struct udc_data udc_data_##n = {                                                    \
		.mutex = Z_MUTEX_INITIALIZER(udc_data_##n.mutex),                                  \
		.priv = &udc_priv_##n,                                                             \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, usb23_driver_preinit, NULL, &udc_data_##n, &usb23_config_##n,     \
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &usb23_api);

DT_INST_FOREACH_STATUS_OKAY(USB23_DEVICE_DEFINE)
