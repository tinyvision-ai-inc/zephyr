/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2023 tinyVision.ai
 *
 * Public methods available on the USB23 blob driver.
 */

#ifndef ZEPHYR_DRIVERS_USB_UDC_USB23_H
#define ZEPHYR_DRIVERS_USB_UDC_USB23_H

#define CONFIG_USB23_EVT_NUM 64

/*
 * Structure for holding controller configuration items that can remain in
 * non-volatile memory.
 */
struct usb23_config {
	/* USB endpoints configuration and data */
	size_t num_bidir_eps;
	struct usb23_ep_data *ep_data;
	struct udc_ep_config *ep_cfg;

	/* USB device configuration */
	int speed_idx;

	/* Base address at which the USB23 core registers are mapped */
	uintptr_t base;

	/* Base address of a memory region that ignores all write requests */
	uintptr_t discard;

	/* Functions pointers for managing the IRQs from LiteX */
	void (*irq_enable_func)(void);
	void (*irq_clear_func)(void);

	/* Pointers to event buffer fetched by USB23 with DMA */
	volatile union usb23_evt *evt_buf;
};

/*
 * All data specific to one endpoint
 */
struct usb23_ep_data {
	/* Whether the trb_buf refers to an instance of the USB Manager
	 * which will poll an endpoint instead of memory. */
	bool is_usb_manager;

	/* Pointer to the DMA-accessible buffer of TRBs and its size */
	struct usb23_trb *trb_buf;

	/* Size of the TRB ring configured in the DeviceTree */
	size_t num_of_trbs;

	/* Buffer of pointers to net_bufs, matching the TRB buffers */
	struct net_buf **net_buf;

	/* Index of the next TRB to receive data in the TRB ring, Link TRB excluded */
	size_t tail;
	size_t head;

	/* Given by the hardware for use in endpoint commands */
	uint32_t xferrscidx;
};

/*
 * Data of each instance of the driver, that can be read and written to.
 * Accessed via "udc_get_private(dev)".
 */
struct usb23_data {
	/* Index within trb where to queue new TRBs */
	uint8_t evt_next;

	/* A work queue entry to process the events from the event buffer */
	struct k_work work;

	/* Back-reference to parent */
	const struct device *dev;

	/* Size of the data stage of control transaction */
	size_t data_stage_length;
};

/*
 * Format for an endpoint-specific event.
 */
struct usb23_depevt {
	uint32_t category: 1;
	uint32_t ep_num: 5;
	uint32_t type: 4;
	uint32_t reserved_11_10: 2;
	uint32_t status: 4;
	uint32_t parameters: 16;
} __packed;

/*
 * Format for a device event unrelated to any endpoint.
 */
struct usb23_devt {
	uint32_t category: 1;
	uint32_t event: 7;
	uint32_t type: 4;
	uint32_t reserved_15_12: 4;
	uint32_t evtinfo: 9;
	uint32_t reserved_31_25: 7;
} __packed;

/*
 * Format of each entry of the event buffer.
 */
union usb23_evt {
	struct usb23_depevt depevt;
	struct usb23_devt devt;
	uint32_t raw;
};

/*
 * Hardware format representing one DMA transaction request passed from the
 * CPU to the USB23 core.
 */
struct usb23_trb {
	uint32_t addr_lo;
	uint32_t addr_hi;
	uint32_t status;
	uint32_t ctrl;
} __packed;

/*
 * Enum matching the device-speed devicetree property elements.
 */
enum {
	USB23_SPEED_IDX_SUPER_SPEED = 3,
	USB23_SPEED_IDX_HIGH_SPEED = 2,
	USB23_SPEED_IDX_FULL_SPEED = 1,
};

/*
 * This is a mapping between logical and physical resources we decoreed.
 * Convert from USB standard endpoint number to physical resource number.
 * It alternates between OUT endpoints (0x00) and IN endpoints (0x80).
 * From: 0x00, 0x80, 0x01, 0x81, 0x02, 0x82, 0x03, 0x83, ...
 * To:   0,    1,    2,    3,    4,    5,    6,    7,    ...
 */
static inline int usb23_get_epn(uint8_t addr)
{
	return ((addr & 0x7f) << 1) | ((addr & 0x80) >> 7);
}

static inline uint8_t usb23_get_addr(int epn)
{
	return (((epn & 0xfe) >> 1) | (epn & 0x01) << 7);
}


static inline struct udc_ep_config *usb23_get_ep_cfg(const struct device *dev, int epn)
{
	const struct usb23_config *config = dev->config;

	return &config->ep_cfg[epn];
}

static inline struct usb23_ep_data *usb23_get_ep_data(const struct device *dev,
					       struct udc_ep_config *const ep_cfg)
{
	const struct usb23_config *config = dev->config;
	int epn = usb23_get_epn(ep_cfg->addr);

	__ASSERT_NO_MSG(epn < config->num_bidir_eps * 2);
	return &config->ep_data[epn];
}

void usb23_on_event(const struct device *dev);
enum udc_bus_speed usb23_api_device_speed(const struct device *dev);
void usb23_enable(const struct device *dev);
int usb23_api_ep_clear_halt(const struct device *dev, struct udc_ep_config *const ep_cfg);
int usb23_api_ep_dequeue(const struct device *dev, struct udc_ep_config *const ep_cfg);
int usb23_api_ep_disable(const struct device *dev, struct udc_ep_config *const ep_cfg);
int usb23_api_ep_enable(const struct device *dev, struct udc_ep_config *const ep_cfg);
int usb23_api_ep_enqueue(const struct device *dev, struct udc_ep_config *const ep_cfg,
			 struct net_buf *buf);
int usb23_api_ep_set_halt(const struct device *dev, struct udc_ep_config *ep_cfg);
int usb23_api_init(const struct device *dev);
int usb23_api_set_address(const struct device *dev, const uint8_t addr);
int usb23_api_set_exit_latency(const struct device *dev, const struct udc_exit_latency *el);
void usb23_irq_handler(void *ptr);

#endif // ZEPHYR_DRIVERS_USB_UDC_USB23_H
