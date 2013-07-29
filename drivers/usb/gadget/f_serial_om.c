/*
 * f_serial_om.c - generic USB serial function driver
 */

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/device.h>

#include "u_serial.h"
#include "gadget_chips.h"


/*
 * This function packages a simple "generic serial" port with no real
 * control mechanisms, just raw data transfer over two bulk endpoints.
 *
 * Because it's not standardized, this isn't as interoperable as the
 * CDC ACM driver.  However, for many purposes it's just as functional
 * if you can arrange appropriate host side drivers.
 */

struct gser_om_descs {
	struct usb_endpoint_descriptor	*in;
	struct usb_endpoint_descriptor	*out;
};

struct f_gser_om {
	struct gserial			port;
	u8				data_id;
	u8				port_num;

	struct gser_om_descs		fs;
	struct gser_om_descs		hs;
	struct usb_cdc_line_coding	port_line_coding;	/* 8-N-1 etc */
	u16				port_handshake_bits;
	struct work_struct  work;
	u32    uevent_action;
};

static inline struct f_gser_om *func_to_gser_om(struct usb_function *f)
{
	return container_of(f, struct f_gser_om, port.func);
}

/*-------------------------------------------------------------------------*/

/* interface descriptor: */

static struct usb_interface_descriptor gser_om_interface_desc  = {
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	/* .bInterfaceNumber = DYNAMIC */
	.bNumEndpoints =	2,
	.bInterfaceClass =	USB_CLASS_CDC_DATA,
	/*.bInterfaceSubClass =	0,*/
	/*.bInterfaceSubClass =	0,*/
	.bInterfaceSubClass =	0,
	.bInterfaceProtocol =	0,
	/* .iInterface = DYNAMIC */
};

static struct usb_cdc_header_desc gser_om_header_desc = {
	.bLength =		sizeof(gser_om_header_desc),
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_HEADER_TYPE,
	.bcdCDC =		cpu_to_le16(0x0110),
};

static struct usb_cdc_call_mgmt_descriptor gser_om_call_mgmt_descriptor = {
	.bLength =		sizeof(gser_om_call_mgmt_descriptor),
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_CALL_MANAGEMENT_TYPE,
	.bmCapabilities =	0,
	/*.bDataInterface = 0 ,*/
};

static struct usb_cdc_acm_descriptor gser_om_descriptor = {
	.bLength =		sizeof(gser_om_descriptor),
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_ACM_TYPE,
	.bmCapabilities =	USB_CDC_CAP_LINE,
};

static struct usb_cdc_union_desc gser_om_union_desc = {
	.bLength =		sizeof(gser_om_union_desc),
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_UNION_TYPE,
	/*.bMasterInterface0 =	0,*/
	/*.bSlaveInterface0   =	0,*/
};
/* full speed support: */

static struct usb_endpoint_descriptor gser_om_fs_in_desc  = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor gser_om_fs_out_desc  = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *gser_om_fs_function[]  = {
	(struct usb_descriptor_header *) &gser_om_interface_desc,
	(struct usb_descriptor_header *) &gser_om_header_desc,
	(struct usb_descriptor_header *) &gser_om_call_mgmt_descriptor,
	(struct usb_descriptor_header *) &gser_om_descriptor,
	(struct usb_descriptor_header *) &gser_om_union_desc,
	(struct usb_descriptor_header *) &gser_om_fs_in_desc,
	(struct usb_descriptor_header *) &gser_om_fs_out_desc,
	NULL,
};

/* high speed support: */

static struct usb_endpoint_descriptor gser_om_hs_in_desc  = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(512),
};

static struct usb_endpoint_descriptor gser_om_hs_out_desc  = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(512),
};

static struct usb_descriptor_header *gser_om_hs_function[]  = {
	(struct usb_descriptor_header *) &gser_om_interface_desc,
	(struct usb_descriptor_header *) &gser_om_header_desc,
	(struct usb_descriptor_header *) &gser_om_descriptor,
	(struct usb_descriptor_header *) &gser_om_call_mgmt_descriptor,
	(struct usb_descriptor_header *) &gser_om_union_desc,
	(struct usb_descriptor_header *) &gser_om_hs_in_desc,
	(struct usb_descriptor_header *) &gser_om_hs_out_desc,
	NULL,
};

/* string descriptors: */

static struct usb_string gser_om_string_defs[] = {
	[0].s = "Generic Serial",
	{  } /* end of list */
};

static struct usb_gadget_strings gser_om_string_table = {
	.language =		0x0409,	/* en-us */
	.strings =		gser_om_string_defs,
};

static struct usb_gadget_strings *gser_om_strings[] = {
	&gser_om_string_table,
	NULL,
};

/*-------------------------------------------------------------------------*/

static int gser_om_set_alt(struct usb_function *f, unsigned intf, unsigned alt)
{
	struct f_gser_om		*gser_om = func_to_gser_om(f);
	struct usb_composite_dev *cdev = f->config->cdev;

	/* we know alt == 0, so this is an activation or a reset */

	if (gser_om->port.in->driver_data) {
		DBG(cdev, "reset generic ttyGS%d\n", gser_om->port_num);
		gserial_disconnect(&gser_om->port);
	} else {
		DBG(cdev, "activate generic ttyGS%d\n", gser_om->port_num);
		gser_om->port.in_desc = ep_choose(cdev->gadget,
				gser_om->hs.in, gser_om->fs.in);
		gser_om->port.out_desc = ep_choose(cdev->gadget,
				gser_om->hs.out, gser_om->fs.out);
	}
	gserial_connect(&gser_om->port, gser_om->port_num);
	return 0;
}


static void gser_om_complete_set_line_coding(struct usb_ep *ep,
		struct usb_request *req)
{
	struct f_gser_om   *gser_om = ep->driver_data;
	struct usb_composite_dev *cdev = gser_om->port.func.config->cdev;


	if (req->status != 0) {
		DBG(cdev, "gser_om ttyGS%d completion, err %d\n",
				gser_om->port_num, req->status);
		return;
	}

	/* normal completion */
	if (req->actual != sizeof(gser_om->port_line_coding)) {
		DBG(cdev, "gser_om ttyGS%d short resp, len %d\n",
				gser_om->port_num, req->actual);
		usb_ep_set_halt(ep);
	} else {
		struct usb_cdc_line_coding *value = req->buf;

		/* REVISIT:  we currently just remember this data.
		 * If we change that, (a) validate it first, then
		 * (b) update whatever hardware needs updating,
		 * (c) worry about locking.  This is information on
		 * the order of 9600-8-N-1 ... most of which means
		 * nothing unless we control a real RS232 line.
		 */
		gser_om->port_line_coding = *value;
	}
}

extern void send_usb_cdc_uevent(int uevent);
static void f_serial_work_func(struct work_struct *work)
{
	struct f_gser_om *gser_om = container_of(work,struct f_gser_om,work);
	send_usb_cdc_uevent(gser_om->uevent_action);
}

static int gser_om_setup(struct usb_function *f, const struct usb_ctrlrequest *ctrl)
{
	struct f_gser_om  *gser_om = func_to_gser_om(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	struct usb_request	*req = cdev->req;
	int			value = -EOPNOTSUPP;
	u16			w_index = le16_to_cpu(ctrl->wIndex);
	u16			w_value = le16_to_cpu(ctrl->wValue);
	u16			w_length = le16_to_cpu(ctrl->wLength);
	switch ((ctrl->bRequestType << 8) | ctrl->bRequest) {

	/* SET_LINE_CODING ... just read and save what the host sends */
	case ((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
			| USB_CDC_REQ_SET_LINE_CODING:
		if (w_length != sizeof(struct usb_cdc_line_coding))
			goto invalid;

		value = w_length;
		cdev->gadget->ep0->driver_data = gser_om;
		req->complete = gser_om_complete_set_line_coding;
		break;

	/* GET_LINE_CODING ... return what host sent, or initial value */
	case ((USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
			| USB_CDC_REQ_GET_LINE_CODING:

		value = min_t(unsigned, w_length,
				sizeof(struct usb_cdc_line_coding));
		memcpy(req->buf, &gser_om->port_line_coding, value);
		break;

	/* SET_CONTROL_LINE_STATE ... save what the host sent */
	case ((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
			| USB_CDC_REQ_SET_CONTROL_LINE_STATE:

		if(w_value == 0x02)
	    {
	       	gser_om->uevent_action = KOBJ_OFFLINE;
	       	schedule_work(&gser_om->work);
	    }
		else if(w_value == 0x00)
		{
			gser_om->uevent_action = KOBJ_REMOVE;
	       	schedule_work(&gser_om->work);
		}

		value = 0;
		gser_om->port_handshake_bits = w_value;
		break;

	default:
invalid:
		VDBG(cdev, "invalid control req%02x.%02x v%04x i%04x l%d\n",
			ctrl->bRequestType, ctrl->bRequest,
			w_value, w_index, w_length);
	}

	/* respond with data transfer or status phase? */
	if (value >= 0) {
		DBG(cdev,  "gser_om  ttyGS%d req%02x.%02x v%04x i%04x l%d\n",
			gser_om->port_num, ctrl->bRequestType, ctrl->bRequest,
			w_value, w_index, w_length);
		req->zero = 0;
		req->length = value;

		value = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
		if (value < 0)
			DBG(cdev,  "gser_om response on ttyGS%d, err %d\n",
					gser_om->port_num, value);
	}

	/* device either stalls (value < 0) or reports success */
	return value;
}

static void gser_om_disable(struct usb_function *f)
{
	struct f_gser_om  *gser_om = func_to_gser_om(f);
	struct usb_composite_dev *cdev = f->config->cdev;

	DBG(cdev, "generic ttyGS%d deactivated\n", gser_om->port_num);
	gserial_disconnect(&gser_om->port);
}

/*-------------------------------------------------------------------------*/

/* serial function driver setup/binding */

static int
gser_om_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct f_gser_om   *gser_om = func_to_gser_om(f);
	int			status;
	struct usb_ep		*ep;

	/* allocate instance-specific interface IDs */
	status = usb_interface_id(c, f);
	if (status < 0)
		goto fail;
	gser_om->data_id = status;
	gser_om_interface_desc.bInterfaceNumber = status;
	gser_om_call_mgmt_descriptor.bDataInterface = status;

	status = -ENODEV;

	/* allocate instance-specific endpoints */
	ep = usb_ep_autoconfig(cdev->gadget, &gser_om_fs_in_desc);
	if (!ep)
		goto fail;
	gser_om->port.in = ep;
	ep->driver_data = cdev;	/* claim */

	ep = usb_ep_autoconfig(cdev->gadget, &gser_om_fs_out_desc);
	if (!ep)
		goto fail;
	gser_om->port.out = ep;
	ep->driver_data = cdev;	/* claim */

	/* copy descriptors, and track endpoint copies */
	f->descriptors = usb_copy_descriptors(gser_om_fs_function);

	if (!f->descriptors) {
		goto fail;
	}

	gser_om->fs.in = usb_find_endpoint(gser_om_fs_function,
			f->descriptors, &gser_om_fs_in_desc);
	gser_om->fs.out = usb_find_endpoint(gser_om_fs_function,
			f->descriptors, &gser_om_fs_out_desc);


	/* support all relevant hardware speeds... we expect that when
	 * hardware is dual speed, all bulk-capable endpoints work at
	 * both speeds
	 */
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		gser_om_hs_in_desc.bEndpointAddress =
				gser_om_fs_in_desc.bEndpointAddress;
		gser_om_hs_out_desc.bEndpointAddress =
				gser_om_fs_out_desc.bEndpointAddress;

		/* copy descriptors, and track endpoint copies */
		f->hs_descriptors = usb_copy_descriptors(gser_om_hs_function);

		gser_om->hs.in = usb_find_endpoint(gser_om_hs_function,
				f->hs_descriptors, &gser_om_hs_in_desc);
		gser_om->hs.out = usb_find_endpoint(gser_om_hs_function,
				f->hs_descriptors, &gser_om_hs_out_desc);
	}

	DBG(cdev, "generic om ttyGS%d: %s speed IN/%s OUT/%s\n",
			gser_om->port_num,
			gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
			gser_om->port.in->name, gser_om->port.out->name);
	printk(KERN_INFO "generic om ttyGS%d: %s speed IN/%s OUT/%s\n",
			gser_om->port_num,
			gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
			gser_om->port.in->name, gser_om->port.out->name);

	return 0;

fail:
	/* we might as well release our claims on endpoints */
	if (gser_om->port.out)
		gser_om->port.out->driver_data = NULL;
	if (gser_om->port.in)
		gser_om->port.in->driver_data = NULL;

	DBG(cdev, "%s: can't bind, err %d\n", f->name, status);

	return status;
}

static void
gser_om_unbind(struct usb_configuration *c, struct usb_function *f)
{
	if (gadget_is_dualspeed(c->cdev->gadget))
		usb_free_descriptors(f->hs_descriptors);
	usb_free_descriptors(f->descriptors);
	kfree(func_to_gser_om(f));
}

/**
 * gser_bind_config - add a generic serial function to a configuration
 * @c: the configuration to support the serial instance
 * @port_num: /dev/ttyGS* port this interface will use
 * Context: single threaded during gadget setup
 *
 * Returns zero on success, else negative errno.
 *
 * Caller must have called @gserial_setup() with enough ports to
 * handle all the ones it binds.  Caller is also responsible
 * for calling @gserial_cleanup() before module unload.
 */
int gser_om_bind_config(struct usb_configuration *c, u8 port_num)
{
	struct f_gser_om   *gser_om;
	int		status;

	/* REVISIT might want instance-specific strings to help
	 * distinguish instances ...
	 */

	/* maybe allocate device-global string ID */
	if (gser_om_string_defs[0].id == 0) {
		status = usb_string_id(c->cdev);
		if (status < 0)
			return status;
		gser_om_string_defs[0].id = status;
	}

	/* allocate and initialize one new instance */
	gser_om = kzalloc(sizeof *gser_om, GFP_KERNEL);
	if (!gser_om)
		return -ENOMEM;

	gser_om->port_num = port_num;
	gser_om->port.func.name = "gser";

	gser_om->port.func.strings = gser_om_strings;
	gser_om->port.func.bind = gser_om_bind;
	gser_om->port.func.unbind = gser_om_unbind;
	gser_om->port.func.set_alt = gser_om_set_alt;
	gser_om->port.func.disable = gser_om_disable;
	INIT_WORK(&gser_om->work, f_serial_work_func);
	gser_om->port.func.setup = gser_om_setup;

	status = usb_add_function(c, &gser_om->port.func);
	if (status)
		kfree(gser_om);
	return status;
}
