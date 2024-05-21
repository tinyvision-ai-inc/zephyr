/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_hid_device

#include "usbd_hid_internal.h"

#include <stdlib.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usbd_hid.h>
#include <zephyr/drivers/usb/udc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(usbd_hid, CONFIG_USBD_HID_LOG_LEVEL);

#define HID_GET_IDLE_DURATION(wValue)		((wValue) >> 8)
#define HID_GET_IDLE_ID(wValue)			(wValue)
#define HID_GET_REPORT_TYPE(wValue)		((wValue) >> 8)
#define HID_GET_REPORT_ID(wValue)		(wValue)

enum {
	HID_REPORT_TYPE_INPUT = 1,
	HID_REPORT_TYPE_OUTPUT,
	HID_REPORT_TYPE_FEATURE,
};

#define HID_SUBORDINATE_DESC_NUM		1

struct subordinate_info {
	uint8_t bDescriptorType;
	uint16_t wDescriptorLength;
} __packed;

/* See HID spec. 6.2 Class-Specific Descriptors */
struct hid_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint16_t bcdHID;
	uint8_t bCountryCode;
	uint8_t bNumDescriptors;
	/* At least report subordinate descriptor is required. */
	struct subordinate_info sub[HID_SUBORDINATE_DESC_NUM];
} __packed;

struct usbd_hid_descriptor {
	struct usb_if_descriptor if0;
	struct hid_descriptor hid;
	struct usb_ep_descriptor in_ep;
	struct usb_ep_companion_descriptor in_ep_ssp;
	struct usb_ep_descriptor out_ep;
	struct usb_ep_companion_descriptor out_ep_ssp;

	struct usb_if_descriptor if0_1;
	struct usb_ep_descriptor alt_hs_in_ep;
	struct usb_ep_descriptor alt_hs_out_ep;
};

enum {
	HID_DEV_CLASS_ENABLED,
};

struct hid_device_data {
	struct usbd_hid_descriptor *const desc;
	struct usbd_class_node *c_nd;
	struct net_buf_pool *const pool_out;
	struct net_buf_pool *const pool_in;
	const struct hid_device_ops *ops;
	const uint8_t *rdesc;
	size_t rsize;
	const struct usb_desc_header **const fs_desc;
	const struct usb_desc_header **const hs_desc;
	atomic_t state;
	struct k_sem in_sem;
	struct k_work output_work;
	uint8_t idle_rate;
	uint8_t protocol;
};

static uint8_t hid_get_in_ep(struct usbd_class_node *const c_nd)
{
	struct usbd_hid_descriptor *desc = c_nd->data->desc;

	return desc->in_ep.bEndpointAddress;
}

static inline uint8_t hid_get_out_ep(struct usbd_class_node *const c_nd)
{
	const struct device *dev = c_nd->data->priv;
	struct hid_device_data *ddata = dev->data;
	struct usbd_hid_descriptor *desc = ddata->desc;

	return desc->out_ep.bEndpointAddress;
}

static int usbd_hid_request(struct usbd_class_node *const c_nd,
			    struct net_buf *const buf, const int err)
{	
	struct usbd_contex *uds_ctx = c_nd->data->uds_ctx;
	const struct device *dev = c_nd->data->priv;
	struct hid_device_data *ddata = dev->data;
	const struct hid_device_ops *ops = ddata->ops;
	struct udc_buf_info *bi;

	bi = udc_get_buf_info(buf);

	if (bi->ep == hid_get_out_ep(c_nd)) {
		if (ops->output_report != NULL) {
			if (err == 0) {
				ops->output_report(dev, buf->len, buf->data);
			}

			k_work_submit(&ddata->output_work);
		}
	}

	if (bi->ep == hid_get_in_ep(c_nd)) {
		if (ops->input_report_done != NULL) {
			ops->input_report_done(dev);
		} else {
			k_sem_give(&ddata->in_sem);
		}
	}

	return usbd_ep_buf_free(uds_ctx, buf);
}

static int handle_set_idle(const struct device *dev,
			   const struct usb_setup_packet *const setup)
{
	const uint32_t duration = HID_GET_IDLE_DURATION(setup->wValue);
	const uint8_t id = HID_GET_IDLE_ID(setup->wValue);
	struct hid_device_data *const ddata = dev->data;
	const struct hid_device_ops *ops = ddata->ops;

	if (id == 0U) {
		/* Only the common idle rate is stored. */
		ddata->idle_rate = duration;
	}

	LOG_DBG("Set Idle, Report ID %u Duration %u", id, duration);

	if (ops->set_idle != NULL) {
		ops->set_idle(dev, id, duration * 4UL);
	} else {
		errno = -ENOTSUP;
	}

	return 0;
}

static int handle_get_idle(const struct device *dev,
			   const struct usb_setup_packet *const setup,
			   struct net_buf *const buf)
{
	const uint8_t id = HID_GET_IDLE_ID(setup->wValue);
	struct hid_device_data *const ddata = dev->data;
	const struct hid_device_ops *ops = ddata->ops;
	uint32_t duration;

	if (setup->wLength != 1U) {
		errno = -ENOTSUP;
		return 0;
	}

	if (id != 0U && ops->get_idle == NULL) {
		errno = -ENOTSUP;
		return 0;
	}

	if (id == 0U) {
		/* Only the common idle rate is stored. */
		duration = ddata->idle_rate;
	} else {
		duration = ops->get_idle(dev, id) / 4UL;
	}

	LOG_DBG("Get Idle, Report ID %u Duration %u", id, duration);
	net_buf_add_u8(buf, duration);

	return 0;
}

static int handle_set_report(const struct device *dev,
			     const struct usb_setup_packet *const setup,
			     const struct net_buf *const buf)
{
	const uint8_t type = HID_GET_REPORT_TYPE(setup->wValue);
	const uint8_t id = HID_GET_REPORT_ID(setup->wValue);
	struct hid_device_data *const ddata = dev->data;
	const struct hid_device_ops *ops = ddata->ops;

	if (ops->set_report == NULL) {
		errno = -ENOTSUP;
		LOG_DBG("Set Report not supported");
		return 0;
	}

	switch (type) {
	case HID_REPORT_TYPE_INPUT:
		LOG_DBG("Set Report, Input Report ID %u", id);
		errno = ops->set_report(dev, type, id, buf->len, buf->data);
		break;
	case HID_REPORT_TYPE_OUTPUT:
		LOG_DBG("Set Report, Output Report ID %u", id);
		errno = ops->set_report(dev, type, id, buf->len, buf->data);
		break;
	case HID_REPORT_TYPE_FEATURE:
		LOG_DBG("Set Report, Feature Report ID %u", id);
		errno = ops->set_report(dev, type, id, buf->len, buf->data);
		break;
	default:
		errno = -ENOTSUP;
		break;
	}

	return 0;
}

static int handle_get_report(const struct device *dev,
			     const struct usb_setup_packet *const setup,
			     struct net_buf *const buf)
{
	const uint8_t type = HID_GET_REPORT_TYPE(setup->wValue);
	const uint8_t id = HID_GET_REPORT_ID(setup->wValue);
	struct hid_device_data *const ddata = dev->data;
	const struct hid_device_ops *ops = ddata->ops;

	switch (type) {
	case HID_REPORT_TYPE_INPUT:
		LOG_DBG("Get Report, Input Report ID %u", id);
		errno = ops->get_report(dev, type, id, net_buf_tailroom(buf), buf->data);
		break;
	case HID_REPORT_TYPE_OUTPUT:
		LOG_DBG("Get Report, Output Report ID %u", id);
		errno = ops->get_report(dev, type, id, net_buf_tailroom(buf), buf->data);
		break;
	case HID_REPORT_TYPE_FEATURE:
		LOG_DBG("Get Report, Feature Report ID %u", id);
		errno = ops->get_report(dev, type, id, net_buf_tailroom(buf), buf->data);
		break;
	default:
		errno = -ENOTSUP;
		break;
	}

	return 0;
}

static int handle_set_protocol(const struct device *dev,
			       const struct usb_setup_packet *const setup)
{
	struct hid_device_data *const ddata = dev->data;
	struct usbd_hid_descriptor *const desc = ddata->desc;
	const struct hid_device_ops *const ops = ddata->ops;
	const uint16_t protocol = setup->wValue;

	if (protocol > HID_PROTOCOL_REPORT) {
		/* Can only be 0 (Boot Protocol) or 1 (Report Protocol). */
		errno = -ENOTSUP;
		return 0;
	}

	if (desc->if0.bInterfaceSubClass == 0) {
		/*
		 * The device does not support the boot protocol and we will
		 * not notify it.
		 */
		errno = -ENOTSUP;
		return 0;
	}

	LOG_DBG("Set Protocol: %s", protocol ? "Report" : "Boot");

	if (ddata->protocol != protocol) {
		ddata->protocol = protocol;

		if (ops->set_protocol) {
			ops->set_protocol(dev, protocol);
		}
	}

	return 0;
}

static int handle_get_protocol(const struct device *dev,
			       const struct usb_setup_packet *const setup,
			       struct net_buf *const buf)
{
	struct hid_device_data *const ddata = dev->data;
	struct usbd_hid_descriptor *const desc = ddata->desc;

	if (setup->wValue != 0 || setup->wLength != 1) {
		errno = -ENOTSUP;
		return 0;
	}

	if (desc->if0.bInterfaceSubClass == 0) {
		/* The device does not support the boot protocol */
		errno = -ENOTSUP;
		return 0;
	}

	LOG_DBG("Get Protocol: %s", ddata->protocol ? "Report" : "Boot");
	net_buf_add_u8(buf, ddata->protocol);

	return 0;
}

static int handle_get_descriptor(const struct device *dev,
				 const struct usb_setup_packet *const setup,
				 struct net_buf *const buf)
{
	struct hid_device_data *const ddata = dev->data;
	uint8_t desc_type = USB_GET_DESCRIPTOR_TYPE(setup->wValue);
	uint8_t desc_idx = USB_GET_DESCRIPTOR_INDEX(setup->wValue);
	struct usbd_hid_descriptor *const desc = ddata->desc;

	switch (desc_type) {
	case USB_DESC_HID_REPORT:
		LOG_DBG("Get descriptor report");
		net_buf_add_mem(buf, ddata->rdesc, MIN(ddata->rsize, setup->wLength));
		break;
	case USB_DESC_HID:
		LOG_DBG("Get descriptor HID");
		net_buf_add_mem(buf, &desc->hid, MIN(desc->hid.bLength, setup->wLength));
		break;
	case USB_DESC_HID_PHYSICAL:
		LOG_DBG("Get descriptor physical %u", desc_idx);
		errno = -ENOTSUP;
		break;
	default:
		errno = -ENOTSUP;
		break;
	}

	return 0;
}

static int usbd_hid_ctd(struct usbd_class_node *const c_nd,
			const struct usb_setup_packet *const setup,
			const struct net_buf *const buf)
{
	const struct device *dev = c_nd->data->priv;
	int ret = 0;

	switch (setup->bRequest) {
	case USB_HID_SET_IDLE:
		ret = handle_set_idle(dev, setup);
		break;
	case USB_HID_SET_REPORT:
		ret = handle_set_report(dev, setup, buf);
		break;
	case USB_HID_SET_PROTOCOL:
		ret = handle_set_protocol(dev, setup);
		break;
	default:
		errno = -ENOTSUP;
		break;
	}

	return ret;
}

static int usbd_hid_cth(struct usbd_class_node *const c_nd,
			const struct usb_setup_packet *const setup,
			struct net_buf *const buf)
{
	const struct device *dev = c_nd->data->priv;
	int ret = 0;

	switch (setup->bRequest) {
	case USB_HID_GET_IDLE:
		ret = handle_get_idle(dev, setup, buf);
		break;
	case USB_HID_GET_REPORT:
		ret = handle_get_report(dev, setup, buf);
		break;
	case USB_HID_GET_PROTOCOL:
		ret = handle_get_protocol(dev, setup, buf);
		break;
	case USB_SREQ_GET_DESCRIPTOR:
		ret = handle_get_descriptor(dev, setup, buf);
		break;
	default:
		errno = -ENOTSUP;
		break;
	}

	return ret;
}

static void usbd_hid_sof(struct usbd_class_node *const c_nd)
{
	const struct device *dev = c_nd->data->priv;
	struct hid_device_data *ddata = dev->data;
	const struct hid_device_ops *const ops = ddata->ops;

	if (ops->sof) {
		ops->sof(dev);
	}
}

static void usbd_hid_enable(struct usbd_class_node *const c_nd)
{
	const struct device *dev = c_nd->data->priv;
	struct hid_device_data *ddata = dev->data;
	const struct hid_device_ops *const ops = ddata->ops;
	struct usbd_hid_descriptor *const desc = ddata->desc;

	atomic_set_bit(&ddata->state, HID_DEV_CLASS_ENABLED);
	ddata->protocol = HID_PROTOCOL_REPORT;
	if (ops->iface_ready && -1 != ops->iface_ready) {
		ops->iface_ready(dev, true);
	}

	if (desc->out_ep.bLength != 0U) {
		k_work_submit(&ddata->output_work);
	}

	LOG_DBG("Configuration enabled");
}

static void usbd_hid_disable(struct usbd_class_node *const c_nd)
{
	const struct device *dev = c_nd->data->priv;
	struct hid_device_data *ddata = dev->data;
	const struct hid_device_ops *const ops = ddata->ops;

	atomic_clear_bit(&ddata->state, HID_DEV_CLASS_ENABLED);
	if (ops->iface_ready) {
		ops->iface_ready(dev, false);
	}

	LOG_DBG("Configuration disabled");
}

static void usbd_hid_suspended(struct usbd_class_node *const c_nd)
{
	const struct device *dev = c_nd->data->priv;

	LOG_DBG("Configuration suspended, device %p", dev);
}

static void usbd_hid_resumed(struct usbd_class_node *const c_nd)
{
	const struct device *dev = c_nd->data->priv;

	LOG_DBG("Configuration resumed, device %p", dev);
}

static void *usbd_hid_get_desc(struct usbd_class_node *const c_nd)
{
	const struct device *dev = c_nd->data->priv;
	struct hid_device_data *ddata = dev->data;

	LOG_DBG("%s %p", __func__, c_nd);
	return ddata->hs_desc;
}

static int usbd_hid_init(struct usbd_class_node *const c_nd)
{
	LOG_DBG("HID class init");

	return 0;
}

static void usbd_hid_shutdown(struct usbd_class_node *const c_nd)
{
	LOG_DBG("HID class shutdown");
}

static struct net_buf *hid_buf_alloc_ext(struct hid_device_data *const ddata,
					 const uint16_t size, void *const data,
					 const uint8_t ep)
{
	struct net_buf *buf = NULL;
	struct udc_buf_info *bi;

	buf = net_buf_alloc_with_data(ddata->pool_in, data, size, K_NO_WAIT);
	if (!buf) {
		return NULL;
	}

	bi = udc_get_buf_info(buf);
	memset(bi, 0, sizeof(struct udc_buf_info));
	bi->ep = ep;

	return buf;
}


static struct net_buf *hid_buf_alloc(struct hid_device_data *const ddata,
				     const uint8_t ep)
{
	struct net_buf *buf = NULL;
	struct udc_buf_info *bi;

	buf = net_buf_alloc(ddata->pool_out, K_NO_WAIT);
	if (!buf) {
		return NULL;
	}

	bi = udc_get_buf_info(buf);
	memset(bi, 0, sizeof(struct udc_buf_info));
	bi->ep = ep;

	return buf;
}

static void hid_dev_output_handler(struct k_work *work)
{
	struct hid_device_data *ddata = CONTAINER_OF(work,
						     struct hid_device_data,
						     output_work);
	struct usbd_class_node *c_nd = ddata->c_nd;
	struct net_buf *buf;


	if (!atomic_test_bit(&ddata->state, HID_DEV_CLASS_ENABLED)) {
		return;
	}

	buf = hid_buf_alloc(ddata, hid_get_out_ep(c_nd));
	if (buf == NULL) {
		LOG_ERR("Failed to allocate buffer");
		return;
	}
	
	usbd_ep_enqueue(c_nd, buf);
}

static int hid_dev_submit_report(const struct device *dev,
				 const uint16_t size, const uint8_t *const report,
				 const bool sync)
{
	struct hid_device_data *const ddata = dev->data;
	const struct hid_device_ops *ops = ddata->ops;
	struct usbd_class_node *c_nd = ddata->c_nd;
	struct usbd_contex *uds_ctx = c_nd->data->uds_ctx;
	struct net_buf *buf;
	int ret;

	if (!atomic_test_bit(&ddata->state, HID_DEV_CLASS_ENABLED)) {
		return -EACCES;
	}

	buf = hid_buf_alloc_ext(ddata, size, (void *)report, hid_get_in_ep(c_nd));
	if (buf == NULL) {
		LOG_ERR("Failed to allocate net_buf");
		return -ENOMEM;
	}

	ret = usbd_ep_enqueue(c_nd, buf);
	if (ret) {
		return usbd_ep_buf_free(uds_ctx, buf);
	}

	if (sync && ops->input_report_done == NULL) {
		k_sem_take(&ddata->in_sem, K_FOREVER);
	}

	return 0;
}

static int hid_dev_register(const struct device *dev,
			    const uint8_t *const rdesc, const uint16_t rsize,
			    const struct hid_device_ops *const ops)
{
	struct hid_device_data *const ddata = dev->data;
	struct usbd_hid_descriptor *const desc = ddata->desc;

	if (atomic_test_bit(&ddata->state, HID_DEV_CLASS_ENABLED)) {
		return -EACCES;
	}

	/* Get Report is required for all HID device types. */
	if (ops == NULL || ops->get_report == NULL) {
		LOG_ERR("get_report callback is missing");
		return -EINVAL;
	}

	/* Set Report is required when an output report is declared. */
	if (desc->out_ep.bLength && ops->set_report == NULL) {
		LOG_ERR("set_report callback is missing");
		return -EINVAL;
	}

	/*
	 * Get/Set Protocol are required when device supports boot interface.
	 * Get Protocol is handled internally, no callback is required.
	 */
	if (desc->if0.bInterfaceSubClass && ops->set_protocol == NULL) {
		LOG_ERR("set_protocol callback is missing");
		return -EINVAL;
	}

	ddata->rdesc = rdesc;
	ddata->rsize = rsize;
	ddata->ops = ops;

	sys_put_le16(ddata->rsize, (uint8_t *)&(desc->hid.sub[0].wDescriptorLength));

	LOG_DBG("%s %p done", __func__, dev);
	return 0;
}

static int hid_device_init(const struct device *dev)
{
	struct hid_device_data *const ddata = dev->data;

	k_work_init(&ddata->output_work, hid_dev_output_handler);
	LOG_DBG("HID device %p init", dev);

	return 0;
}

struct usbd_class_api usbd_hid_api = {
	.request = usbd_hid_request,
	.update = NULL,
	.sof = usbd_hid_sof,
	.enable = usbd_hid_enable,
	.disable = usbd_hid_disable,
	.suspended = usbd_hid_suspended,
	.resumed = usbd_hid_resumed,
	.control_to_dev = usbd_hid_ctd,
	.control_to_host = usbd_hid_cth,
	//.get_desc = usbd_hid_get_desc,
	.init = usbd_hid_init,
	.shutdown = usbd_hid_shutdown,
};

static const struct hid_device_driver_api hid_device_api = {
	.submit_report = hid_dev_submit_report,
	.register_app = hid_dev_register,
};

#include "usbd_hid_macros.h"

#define USBD_HID_INTERFACE_SIMPLE_DEFINE(n)					\
	static struct usbd_hid_descriptor hid_desc_##n = {			\
		.if0 = HID_INTERFACE_DEFINE(n, 0),				\
		.in_ep_ssp = HID_IN_EP_SSP_DEFINE(n, false, false),	\
		.hid = HID_DESCRIPTOR_DEFINE(n),				\
		.in_ep = HID_IN_EP_DEFINE(n, true, true),			\
		.out_ep_ssp = HID_OUT_EP_SSP_DEFINE(n, false, false),	\
		.out_ep = HID_OUT_EP_DEFINE_OR_ZERO(n, true, true),		\
	};									\
										\
	const static struct usb_desc_header *hid_hs_desc_##n[] = {		\
		(struct usb_desc_header *) &hid_desc_##n.if0,			\
		(struct usb_desc_header *) &hid_desc_##n.hid,			\
		(struct usb_desc_header *) &hid_desc_##n.in_ep,		\
		(struct usb_desc_header *) &hid_desc_##n.out_ep,		\
		NULL,								\
	}

#define USBD_HID_INTERFACE_ALTERNATE_DEFINE(n)					\
	static struct usbd_hid_descriptor hid_desc_##n = {			\
		.if0 = HID_INTERFACE_DEFINE(n, 0),				\
		.hid = HID_DESCRIPTOR_DEFINE(n),				\
		.in_ep_ssp = HID_IN_EP_SSP_DEFINE(n, false, false),	\
		.in_ep = HID_IN_EP_DEFINE(n, true, false),			\
		.out_ep_ssp = HID_OUT_EP_SSP_DEFINE(n, false, false),	\
		.out_ep = HID_OUT_EP_DEFINE_OR_ZERO(n, true, false),		\
		.if0_1 = HID_INTERFACE_DEFINE(n, 1),				\
		.alt_hs_in_ep = HID_IN_EP_DEFINE(n, true, true),		\
		.alt_hs_out_ep = HID_OUT_EP_DEFINE_OR_ZERO(n, true, true),	\
	};									\
										\
	const static struct usb_desc_header *hid_fs_desc_##n[] = {		\
		(struct usb_desc_header *) &hid_desc_##n.if0,			\
		(struct usb_desc_header *) &hid_desc_##n.hid,			\
		(struct usb_desc_header *) &hid_desc_##n.in_ep,			\
		(struct usb_desc_header *) &hid_desc_##n.out_ep,		\
		NULL,								\
	};									\
										\
	const static struct usb_desc_header *hid_hs_desc_##n[] = {		\
		(struct usb_desc_header *) &hid_desc_##n.if0,			\
		(struct usb_desc_header *) &hid_desc_##n.hid,			\
		(struct usb_desc_header *) &hid_desc_##n.hs_in_ep,		\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, out_report_size),			\
		((struct usb_desc_header *) &hid_desc_##n.hs_out_ep,), ())	\
		(struct usb_desc_header *)&hid_desc_##n.if0_1,			\
		(struct usb_desc_header *) &hid_desc_##n.hid,			\
		(struct usb_desc_header *) &hid_desc_##n.alt_hs_in_ep,		\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, out_report_size),			\
		((struct usb_desc_header *) &hid_desc_##n.alt_hs_out_ep,), ())	\
		NULL,								\
	}


#define USBD_HID_INTERFACE_DEFINE(n)						\
	USBD_HID_INTERFACE_SIMPLE_DEFINE(n)

#define USBD_HID_INSTANCE_DEFINE(n)						\
	NET_BUF_POOL_DEFINE(hid_buf_pool_in_##n,				\
			    CONFIG_USBD_HID_IN_BUF_COUNT, 0,			\
			    sizeof(struct udc_buf_info), NULL);			\
										\
	HID_OUT_POOL_DEFINE(n);							\
	USBD_HID_INTERFACE_DEFINE(n);						\
														\
	static struct usbd_class_data usbd_hid_data_##n;	\
	USBD_DEFINE_CLASS(hid_##n,						\
			  &usbd_hid_api,					\
			  &usbd_hid_data_##n);					\
										\
	static struct hid_device_data hid_data_##n = {				\
		.desc = &hid_desc_##n,						\
		.c_nd = &hid_##n,						\
		.pool_in = &hid_buf_pool_in_##n,				\
		.pool_out = HID_OUT_POOL_ADDR(n),				\
		.in_sem = Z_SEM_INITIALIZER(hid_data_##n.in_sem, 0, 1),		\
	};									\
										\
	static struct usbd_class_data usbd_hid_data_##n = {			\
		.desc = (struct usb_desc_header *)&hid_desc_##n,		\
		.priv = (void *)DEVICE_DT_GET(DT_DRV_INST(n)),			\
	};															\
	DEVICE_DT_INST_DEFINE(n, hid_device_init, NULL,				\
		&hid_data_##n, NULL,						\
		POST_KERNEL, CONFIG_USBD_HID_INIT_PRIORITY,			\
		&hid_device_api);

DT_INST_FOREACH_STATUS_OKAY(USBD_HID_INSTANCE_DEFINE);
