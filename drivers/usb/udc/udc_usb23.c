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

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(usb23, CONFIG_UDC_DRIVER_LOG_LEVEL);

/*
 * USB device controller (UDC) for Lattice USB2 and USB3 hardened FPGA core.
 */
#include "udc_common.h"
#include "udc_usb23.h"

static int usb23_api_enable(const struct device *dev)
{
	LOG_INF("%s", __func__);
	usb23_enable(dev);
	return 0;
}

static int usb23_api_disable(const struct device *dev)
{
	LOG_INF("%s", __func__);
	return 0;
}

static int usb23_api_host_wakeup(const struct device *dev)
{
	return 0;
}

/*
 * Shut down the controller completely
 */
static int usb23_api_shutdown(const struct device *dev)
{
	LOG_INF("%s", __func__);
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

static int usb23_api_lock(const struct device *dev)
{
	return udc_lock_internal(dev, K_FOREVER);
}

static int usb23_api_unlock(const struct device *dev)
{
	return udc_unlock_internal(dev);
}

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

static void usb23_event_worker(struct k_work *work)
{
	const struct usb23_data *priv = CONTAINER_OF(work, struct usb23_data, work);
	const struct device *dev = priv->dev;

	usb23_on_event(dev);
}

/*
 * The real init function is triggered by the application with .
 */
static int usb23_ep_preinit(const struct device *dev, struct udc_ep_config *const ep_cfg,
			    uint8_t addr, int mps)
{
	int ret;

	LOG_DBG("%s: ep=0x%02x ep=%p ctrl=%d", __func__, ep_cfg->addr, ep_cfg,
		ep_cfg->caps.control);

	/* Generic properties for Zephyr */
	ep_cfg->addr = addr;
	if (ep_cfg->addr & USB_EP_DIR_IN) {
		ep_cfg->caps.in = 1;
	} else {
		ep_cfg->caps.out = 1;
	}
	if (ep_cfg->addr == USB_CONTROL_EP_OUT || ep_cfg->addr == USB_CONTROL_EP_IN) {
		LOG_DBG("%s: control", __func__);
		ep_cfg->caps.control = 1;
	} else {
		LOG_DBG("%s: normal", __func__);
		ep_cfg->caps.bulk = 1;
		ep_cfg->caps.interrupt = 1;
		ep_cfg->caps.iso = 1;
	}
	ep_cfg->caps.mps = mps;

	ret = udc_register_ep(dev, ep_cfg);
	if (ret != 0) {
		LOG_ERR("Failed to register endpoint");
		return ret;
	}

	return 0;
}

/*
 * Initialize the controller and endpoints capabilities,
 * register endpoint structures, no hardware I/O yet.
 */
int usb23_driver_preinit(const struct device *dev)
{
	const struct usb23_config *config = dev->config;
	struct udc_data *data = dev->data;
	struct usb23_data *priv = udc_get_private(dev);
	uint16_t mps = 0;

	k_mutex_init(&data->mutex);
	k_work_init(&priv->work, &usb23_event_worker);

	data->caps.rwup = true;
	switch (config->speed_idx) {
	case USB23_SPEED_IDX_SUPER_SPEED:
		data->caps.mps0 = UDC_MPS0_512;
		data->caps.ss = true;
		data->caps.hs = true;
		mps = 1024;
		break;
	case USB23_SPEED_IDX_HIGH_SPEED:
		data->caps.mps0 = UDC_MPS0_64;
		data->caps.hs = true;
		mps = 1024;
		break;
	case USB23_SPEED_IDX_FULL_SPEED:
		data->caps.mps0 = UDC_MPS0_64; // TODO: adjust
		mps = 64;                      // TODO: adjust
		break;
	default:
		LOG_ERR("Not implemented");
	}

	for (int epn = 0; epn < config->num_bidir_eps * 2; epn++) {
		usb23_ep_preinit(dev, usb23_get_ep_cfg(dev, epn), usb23_get_addr(epn), mps);
	}

	return 0;
}

#define USB23_EP_TRB_BUF_DEFINE(n)                                                                 \
	static struct usb23_trb usb23_dma_trb_buf_##n[DT_PROP(n, num_trbs)];

#define USB23_EP_NET_BUF_DEFINE(n) static struct net_buf *usb23_net_buf_##n[DT_PROP(n, num_trbs)];

#define USB23_EP_DATA_ENTRY(n)                                                                     \
	{                                                                                          \
		.is_usb_manager = DT_NODE_HAS_PROP(n, usb_manager),                                \
		.num_of_trbs = DT_PROP(n, num_trbs),                                               \
		.trb_buf = usb23_dma_trb_buf_##n,                                                  \
		.net_buf = usb23_net_buf_##n,                                                      \
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
	struct udc_ep_config usb23_ep_cfg_##n[DT_INST_PROP(n, num_bidir_endpoints) * 2];           \
                                                                                                   \
	DT_FOREACH_CHILD(DT_INST_CHILD(n, endpoints), USB23_EP_TRB_BUF_DEFINE)                     \
	DT_FOREACH_CHILD(DT_INST_CHILD(n, endpoints), USB23_EP_NET_BUF_DEFINE)                     \
                                                                                                   \
	static struct usb23_ep_data usb23_ep_data_##n[] = {                                        \
		DT_FOREACH_CHILD(DT_INST_CHILD(n, endpoints), USB23_EP_DATA_ENTRY)};               \
                                                                                                   \
	static const struct usb23_config usb23_config_##n = {                                      \
		.base = DT_INST_REG_ADDR_BY_NAME(n, base),                                         \
		.discard = DT_INST_REG_ADDR_BY_NAME(n, discard),                                   \
		.num_bidir_eps = DT_INST_PROP(n, num_bidir_endpoints),                             \
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
