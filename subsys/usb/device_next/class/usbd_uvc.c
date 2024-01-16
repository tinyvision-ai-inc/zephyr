/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usb_uvc.h>

#include <zephyr/drivers/usb/udc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(usbd_uvc, CONFIG_USBD_UVC_LOG_LEVEL);

struct usbd_cdc_acm_desc {
	struct usb_association_descriptor iad_cdc;
	struct usb_if_descriptor if0;
	struct cdc_header_descriptor if0_header;
	struct cdc_cm_descriptor if0_cm;
	struct cdc_acm_descriptor if0_acm;
	struct cdc_union_descriptor if0_union;
	struct usb_ep_descriptor if0_int_ep;

	struct usb_if_descriptor if1;
	struct usb_ep_descriptor if1_in_ep;
	struct usb_ep_descriptor if1_out_ep;

	struct usb_desc_header nil_desc;
} __packed;

static void cdc_acm_irq_rx_enable(const struct device *dev);

static ALWAYS_INLINE int cdc_acm_work_submit(struct k_work *work)
{
	return k_work_submit_to_queue(&cdc_acm_work_q, work);
}

static ALWAYS_INLINE bool check_wq_ctx(const struct device *dev)
{
	return k_current_get() == k_work_queue_thread_get(&cdc_acm_work_q);
}

static uint8_t cdc_acm_get_int_in(struct usbd_class_node *const c_nd)
{
	struct usbd_cdc_acm_desc *desc = c_nd->data->desc;

	return desc->if0_int_ep.bEndpointAddress;
}

static uint8_t cdc_acm_get_bulk_in(struct usbd_class_node *const c_nd)
{
	struct usbd_cdc_acm_desc *desc = c_nd->data->desc;

	return desc->if1_in_ep.bEndpointAddress;
}

static uint8_t cdc_acm_get_bulk_out(struct usbd_class_node *const c_nd)
{
	struct usbd_cdc_acm_desc *desc = c_nd->data->desc;

	return desc->if1_out_ep.bEndpointAddress;
}

static size_t cdc_acm_get_bulk_mps(struct usbd_class_node *const c_nd)
{
	struct usbd_cdc_acm_desc *desc = c_nd->data->desc;

	return desc->if1_out_ep.wMaxPacketSize;
}

static int usbd_cdc_acm_request(struct usbd_class_node *const c_nd,
				struct net_buf *buf, int err)
{
	struct usbd_contex *uds_ctx = c_nd->data->uds_ctx;
	const struct device *dev = c_nd->data->priv;
	struct cdc_acm_uart_data *data = dev->data;
	struct udc_buf_info *bi;

	bi = udc_get_buf_info(buf);
	if (err) {
		if (err == -ECONNABORTED) {
			LOG_WRN("request ep 0x%02x, len %u cancelled",
				bi->ep, buf->len);
		} else {
			LOG_ERR("request ep 0x%02x, len %u failed",
				bi->ep, buf->len);
		}

		if (bi->ep == cdc_acm_get_bulk_out(c_nd)) {
			atomic_clear_bit(&data->state, CDC_ACM_RX_FIFO_BUSY);
		}

		goto ep_request_error;
	}

	if (bi->ep == cdc_acm_get_bulk_out(c_nd)) {
		/* RX transfer completion */
		size_t done;

		LOG_HEXDUMP_INF(buf->data, buf->len, "");
		done = ring_buf_put(data->rx_fifo.rb, buf->data, buf->len);
		if (done && data->cb) {
			cdc_acm_work_submit(&data->irq_cb_work);
		}

		atomic_clear_bit(&data->state, CDC_ACM_RX_FIFO_BUSY);
		cdc_acm_work_submit(&data->rx_fifo_work);
	}

	if (bi->ep == cdc_acm_get_bulk_in(c_nd)) {
		/* TX transfer completion */
		if (data->cb) {
			cdc_acm_work_submit(&data->irq_cb_work);
		}
	}

	if (bi->ep == cdc_acm_get_int_in(c_nd)) {
		k_sem_give(&data->notif_sem);
	}

ep_request_error:
	return usbd_ep_buf_free(uds_ctx, buf);
}

static void usbd_cdc_acm_update(struct usbd_class_node *const c_nd,
				uint8_t iface, uint8_t alternate)
{
	LOG_DBG("New configuration, interface %u alternate %u",
		iface, alternate);
}

static void usbd_cdc_acm_enable(struct usbd_class_node *const c_nd)
{
	const struct device *dev = c_nd->data->priv;
	struct cdc_acm_uart_data *data = dev->data;

	atomic_set_bit(&data->state, CDC_ACM_CLASS_ENABLED);
	LOG_INF("Configuration enabled");

	if (atomic_test_bit(&data->state, CDC_ACM_IRQ_RX_ENABLED)) {
		cdc_acm_irq_rx_enable(dev);
	}

	if (atomic_test_bit(&data->state, CDC_ACM_IRQ_TX_ENABLED)) {
		/* TODO */
	}
}

static void usbd_cdc_acm_disable(struct usbd_class_node *const c_nd)
{
	const struct device *dev = c_nd->data->priv;
	struct cdc_acm_uart_data *data = dev->data;

	atomic_clear_bit(&data->state, CDC_ACM_CLASS_ENABLED);
	atomic_clear_bit(&data->state, CDC_ACM_CLASS_SUSPENDED);
	LOG_INF("Configuration disabled");
}

static void usbd_cdc_acm_suspended(struct usbd_class_node *const c_nd)
{
	const struct device *dev = c_nd->data->priv;
	struct cdc_acm_uart_data *data = dev->data;

	/* FIXME: filter stray suspended events earlier */
	atomic_set_bit(&data->state, CDC_ACM_CLASS_SUSPENDED);
}

static void usbd_cdc_acm_resumed(struct usbd_class_node *const c_nd)
{
	const struct device *dev = c_nd->data->priv;
	struct cdc_acm_uart_data *data = dev->data;

	atomic_clear_bit(&data->state, CDC_ACM_CLASS_SUSPENDED);
}

static void cdc_acm_update_uart_cfg(struct cdc_acm_uart_data *const data)
{
	struct uart_config *const cfg = &data->uart_cfg;

	cfg->baudrate = sys_le32_to_cpu(data->line_coding.dwDTERate);

	switch (data->line_coding.bCharFormat) {
	case USB_CDC_LINE_CODING_STOP_BITS_1:
		cfg->stop_bits = UART_CFG_STOP_BITS_1;
		break;
	case USB_CDC_LINE_CODING_STOP_BITS_1_5:
		cfg->stop_bits = UART_CFG_STOP_BITS_1_5;
		break;
	case USB_CDC_LINE_CODING_STOP_BITS_2:
	default:
		cfg->stop_bits = UART_CFG_STOP_BITS_2;
		break;
	};

	switch (data->line_coding.bParityType) {
	case USB_CDC_LINE_CODING_PARITY_NO:
	default:
		cfg->parity = UART_CFG_PARITY_NONE;
		break;
	case USB_CDC_LINE_CODING_PARITY_ODD:
		cfg->parity = UART_CFG_PARITY_ODD;
		break;
	case USB_CDC_LINE_CODING_PARITY_EVEN:
		cfg->parity = UART_CFG_PARITY_EVEN;
		break;
	case USB_CDC_LINE_CODING_PARITY_MARK:
		cfg->parity = UART_CFG_PARITY_MARK;
		break;
	case USB_CDC_LINE_CODING_PARITY_SPACE:
		cfg->parity = UART_CFG_PARITY_SPACE;
		break;
	};

	switch (data->line_coding.bDataBits) {
	case USB_CDC_LINE_CODING_DATA_BITS_5:
		cfg->data_bits = UART_CFG_DATA_BITS_5;
		break;
	case USB_CDC_LINE_CODING_DATA_BITS_6:
		cfg->data_bits = UART_CFG_DATA_BITS_6;
		break;
	case USB_CDC_LINE_CODING_DATA_BITS_7:
		cfg->data_bits = UART_CFG_DATA_BITS_7;
		break;
	case USB_CDC_LINE_CODING_DATA_BITS_8:
	default:
		cfg->data_bits = UART_CFG_DATA_BITS_8;
		break;
	};

	cfg->flow_ctrl = UART_CFG_FLOW_CTRL_NONE;
}

static void cdc_acm_update_linestate(struct cdc_acm_uart_data *const data)
{
	if (data->line_state & SET_CONTROL_LINE_STATE_RTS) {
		data->line_state_rts = true;
	} else {
		data->line_state_rts = false;
	}

	if (data->line_state & SET_CONTROL_LINE_STATE_DTR) {
		data->line_state_dtr = true;
	} else {
		data->line_state_dtr = false;
	}
}

static int usbd_cdc_acm_cth(struct usbd_class_node *const c_nd,
			    const struct usb_setup_packet *const setup,
			    struct net_buf *const buf)
{
	const struct device *dev = c_nd->data->priv;
	struct cdc_acm_uart_data *data = dev->data;
	size_t min_len;

	if (setup->bRequest == GET_LINE_CODING) {
		if (buf == NULL) {
			errno = -ENOMEM;
			return 0;
		}

		min_len = MIN(sizeof(data->line_coding), setup->wLength);
		net_buf_add_mem(buf, &data->line_coding, min_len);

		return 0;
	}

	LOG_DBG("bmRequestType 0x%02x bRequest 0x%02x unsupported",
		setup->bmRequestType, setup->bRequest);
	errno = -ENOTSUP;

	return 0;
}

static int usbd_cdc_acm_ctd(struct usbd_class_node *const c_nd,
			    const struct usb_setup_packet *const setup,
			    const struct net_buf *const buf)
{
	const struct device *dev = c_nd->data->priv;
	struct cdc_acm_uart_data *data = dev->data;
	size_t len;

	switch (setup->bRequest) {
	case SET_LINE_CODING:
		len = sizeof(data->line_coding);
		if (setup->wLength != len) {
			errno = -ENOTSUP;
			return 0;
		}

		memcpy(&data->line_coding, buf->data, len);
		cdc_acm_update_uart_cfg(data);
		return 0;

	case SET_CONTROL_LINE_STATE:
		data->line_state = setup->wValue;
		cdc_acm_update_linestate(data);
		return 0;

	default:
		break;
	}

	LOG_DBG("bmRequestType 0x%02x bRequest 0x%02x unsupported",
		setup->bmRequestType, setup->bRequest);
	errno = -ENOTSUP;

	return 0;
}

static int usbd_cdc_acm_init(struct usbd_class_node *const c_nd)
{
	struct usbd_cdc_acm_desc *desc = c_nd->data->desc;

	desc->iad_cdc.bFirstInterface = desc->if0.bInterfaceNumber;
	desc->if0_union.bControlInterface = desc->if0.bInterfaceNumber;
	desc->if0_union.bSubordinateInterface0 = desc->if1.bInterfaceNumber;

	return 0;
}

static int usbd_uvc_preinit(const struct device *dev)
{
	struct cdc_acm_uart_data *const data = dev->data;

	ring_buf_reset(data->tx_fifo.rb);
	ring_buf_reset(data->rx_fifo.rb);

	k_thread_name_set(&cdc_acm_work_q.thread, "cdc_acm_work_q");

	k_work_init(&data->tx_fifo_work, cdc_acm_tx_fifo_handler);
	k_work_init(&data->rx_fifo_work, cdc_acm_rx_fifo_handler);
	k_work_init(&data->irq_cb_work, cdc_acm_irq_cb_handler);

	return 0;
}

struct usbd_class_api usbd_uvc_api = {
	.request = usbd_uvc_request,
	.update = usbd_uvc_update,
	.enable = usbd_uvc_enable,
	.disable = usbd_uvc_disable,
	.suspended = usbd_uvc_suspended,
	.resumed = usbd_uvc_resumed,
	.control_to_host = usbd_uvc_cth,
	.control_to_dev = usbd_uvc_ctd,
	.init = usbd_uvc_init,
};

#define UVC_DEFINE_DESCRIPTOR(n)						\
static struct usbd_uvc_desc uvc_desc_##n = {					\
	.iad_cdc = {								\
		.bLength = sizeof(struct usb_association_descriptor),		\
		.bDescriptorType = USB_DESC_INTERFACE_ASSOC,			\
		.bFirstInterface = 0,						\
		.bInterfaceCount = 0x02,					\
		.bFunctionClass = USB_BCC_CDC_CONTROL,				\
		.bFunctionSubClass = ACM_SUBCLASS,				\
		.bFunctionProtocol = 0,						\
		.iFunction = 0,							\
	},									\
										\
	.if0 = {								\
		.bLength = sizeof(struct usb_if_descriptor),			\
		.bDescriptorType = USB_DESC_INTERFACE,				\
		.bInterfaceNumber = 0,						\
		.bAlternateSetting = 0,						\
		.bNumEndpoints = 1,						\
		.bInterfaceClass = USB_BCC_CDC_CONTROL,				\
		.bInterfaceSubClass = ACM_SUBCLASS,				\
		.bInterfaceProtocol = 0,					\
		.iInterface = 0,						\
	},									\
										\
	.if0_header = {								\
		.bFunctionLength = sizeof(struct cdc_header_descriptor),	\
		.bDescriptorType = USB_DESC_CS_INTERFACE,			\
		.bDescriptorSubtype = HEADER_FUNC_DESC,				\
		.bcdCDC = sys_cpu_to_le16(USB_SRN_1_1),				\
	},									\
										\
	.if0_cm = {								\
		.bFunctionLength = sizeof(struct cdc_cm_descriptor),		\
		.bDescriptorType = USB_DESC_CS_INTERFACE,			\
		.bDescriptorSubtype = CALL_MANAGEMENT_FUNC_DESC,		\
		.bmCapabilities = 0,						\
		.bDataInterface = 1,						\
	},									\
										\
	.if0_acm = {								\
		.bFunctionLength = sizeof(struct cdc_acm_descriptor),		\
		.bDescriptorType = USB_DESC_CS_INTERFACE,			\
		.bDescriptorSubtype = ACM_FUNC_DESC,				\
		/* See CDC PSTN Subclass Chapter 5.3.2 */			\
		.bmCapabilities = BIT(1),					\
	},									\
										\
	.if0_union = {								\
		.bFunctionLength = sizeof(struct cdc_union_descriptor),		\
		.bDescriptorType = USB_DESC_CS_INTERFACE,			\
		.bDescriptorSubtype = UNION_FUNC_DESC,				\
		.bControlInterface = 0,						\
		.bSubordinateInterface0 = 1,					\
	},									\
										\
	.if0_int_ep = {								\
		.bLength = sizeof(struct usb_ep_descriptor),			\
		.bDescriptorType = USB_DESC_ENDPOINT,				\
		.bEndpointAddress = 0x81,					\
		.bmAttributes = USB_EP_TYPE_INTERRUPT,				\
		.wMaxPacketSize = sys_cpu_to_le16(CDC_ACM_DEFAULT_INT_EP_MPS),	\
		.bInterval = CDC_ACM_DEFAULT_INT_INTERVAL,			\
	},									\
										\
	.if1 = {								\
		.bLength = sizeof(struct usb_if_descriptor),			\
		.bDescriptorType = USB_DESC_INTERFACE,				\
		.bInterfaceNumber = 1,						\
		.bAlternateSetting = 0,						\
		.bNumEndpoints = 2,						\
		.bInterfaceClass = USB_BCC_CDC_DATA,				\
		.bInterfaceSubClass = 0,					\
		.bInterfaceProtocol = 0,					\
		.iInterface = 0,						\
	},									\
										\
	.if1_in_ep = {								\
		.bLength = sizeof(struct usb_ep_descriptor),			\
		.bDescriptorType = USB_DESC_ENDPOINT,				\
		.bEndpointAddress = 0x82,					\
		.bmAttributes = USB_EP_TYPE_BULK,				\
		.wMaxPacketSize = sys_cpu_to_le16(CDC_ACM_DEFAULT_BULK_EP_MPS),	\
		.bInterval = 0,							\
	},									\
										\
	.if1_out_ep = {								\
		.bLength = sizeof(struct usb_ep_descriptor),			\
		.bDescriptorType = USB_DESC_ENDPOINT,				\
		.bEndpointAddress = 0x01,					\
		.bmAttributes = USB_EP_TYPE_BULK,				\
		.wMaxPacketSize = sys_cpu_to_le16(CDC_ACM_DEFAULT_BULK_EP_MPS),	\
		.bInterval = 0,							\
	},									\
										\
	.nil_desc = {								\
		.bLength = 0,							\
		.bDescriptorType = 0,						\
	},									\
}

#define DT_DRV_COMPAT zephyr_uvc

#define USBD_UVC_DT_DEVICE_DEFINE(n)						\
	BUILD_ASSERT(DT_INST_ON_BUS(n, usb),					\
		     "node " DT_NODE_PATH(DT_DRV_INST(n))			\
		     " is not assigned to a USB device controller");		\
										\
	UVC_DEFINE_DESCRIPTOR(n);						\
										\
	static struct usbd_class_data usbd_cdc_acm_data_##n;			\
										\
	USBD_DEFINE_CLASS(cdc_acm_##n,						\
			  &usbd_cdc_acm_api,					\
			  &usbd_cdc_acm_data_##n);				\
										\
	static struct cdc_acm_uart_data uart_data_##n = {			\
		.line_coding = CDC_ACM_DEFAULT_LINECODING,			\
		.c_nd = &cdc_acm_##n,						\
		.rx_fifo.rb = &cdc_acm_rb_rx_##n,				\
		.tx_fifo.rb = &cdc_acm_rb_tx_##n,				\
		.notif_sem = Z_SEM_INITIALIZER(uart_data_##n.notif_sem, 0, 1),	\
	};									\
										\
	static struct usbd_class_data usbd_cdc_acm_data_##n = {			\
		.desc = (struct usb_desc_header *)&cdc_acm_desc_##n,		\
		.priv = (void *)DEVICE_DT_GET(DT_DRV_INST(n)),			\
	};									\


DT_INST_FOREACH_STATUS_OKAY(USBD_UVC_DT_DEVICE_DEFINE);
