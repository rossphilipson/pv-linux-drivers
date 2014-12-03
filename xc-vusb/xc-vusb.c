/******************************************************************************
 * vusb.c
 *
 * OpenXT vUSB frontend driver
 *
 * Copyright (c) 2013 Julien Grall
 * Copyright (c) 2011 Thomas Horsten
 * Copyright (c) 2013 OpenXT Systems, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/***
 *** TODO
 *** - Modify thread handling
 *** - Add branch prediction
 *** - Send cancel URB command if needed
 *** - Management support
 *** - Devices are not kept accross suspend/hibernate (vusb daemon issue)
 *** - Reorganize the code
 ***/

#include <linux/mm.h>
#include <linux/version.h>
#include <linux/module.h>
#include <asm/uaccess.h>
#include <linux/kthread.h>
#include <linux/signal.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/usb.h>
#if ( LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0) )
#include <linux/aio.h>
#endif

#include <xen/xen.h>
#include <xen/xenbus.h>
#include <xen/events.h>
#include <xen/page.h>
#include <xen/grant_table.h>

#include <xen/interface/io/usbif.h>
#include <xen/interface/memory.h>
#include <xen/interface/grant_table.h>

/* TODO looks like we use some old HCD interface, update */
#if ( LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35) || (defined(RHEL_RELEASE_CODE)) )
#include <linux/usb/hcd.h>
#else
#include <linux/old-core-hcd.h>
#endif

#define GRANT_INVALID_REF 0

#define VUSB_INTERFACE_VERSION 	3
#define VUSB_MAX_PACKET_SIZE 	1024*256

#define VUSB_PLATFORM_DRIVER_NAME	"vusb-platform"
#define VUSB_HCD_DRIVER_NAME		"vusb-hcd"
#define VUSB_DRIVER_DESC		"OpenXT Virtual USB Host Controller"
#define VUSB_DRIVER_VERSION		"1.0.0"

#define POWER_BUDGET	5000 /* mA */

#define D_VUSB1 (1 << 0)
#define D_VUSB2 (1 << 1)
#define D_URB1  (1 << 2)
#define D_URB2  (1 << 3)
#define D_STATE (1 << 4)
#define D_PORT1 (1 << 5)
#define D_PORT2 (1 << 6)
#define D_CTRL (1 << 8)
#define D_MISC (1 << 9)
#define D_WARN (1 << 10) /* only debugging warn */
#define D_PM (1 << 11)
#define D_RING1 (1 << 12)
#define D_RING2 (1 << 13)

#define DEBUGMASK (D_STATE | D_PORT1 | D_URB1 | D_PM)

/* #define VUSB_DEBUG */

#ifdef VUSB_DEBUG
#  define dprintk(mask, args...)					\
	do {								\
		if (DEBUGMASK & mask)					\
			printk(KERN_DEBUG "vusb: "args);		\
	} while (0)

#  define dprint_hex_dump(mask, args...)				\
	do {								\
		if (DEBUGMASK & mask)					\
			print_hex_dump(KERN_DEBUG, "vusb: "args);	\
	} while (0)
#else
#  define dprintk(args...) do {} while (0)
#  define dprint_hex_dump(args...) do {} while (0)
#endif

#define eprintk(args...) printk(KERN_ERR "vusb: "args)
#define wprintk(args...) printk(KERN_WARNING "vusb: "args)
#define iprintk(args...) printk(KERN_INFO "vusb: "args)

/* How many ports on the root hub */
#ifdef VUSB_DEBUG
# define VUSB_PORTS	2
#else
# define VUSB_PORTS    USB_MAXCHILDREN
#endif

/* Status codes */
#define VUSB_URB_STATUS_SUCCESS            0x00000000
#define VUSB_URB_STATUS_FAILURE            0xFFFFFFFF

/* Command flag aliases for USB kernel URB states */
#define VUSB_URB_DIRECTION_IN      0x0001
#define VUSB_URB_SHORT_OK          0x0002
#define VUSB_URB_ISO_TRANSFER_ASAP 0x0004

#if ( LINUX_VERSION_CODE >= KERNEL_VERSION(3,6,0) )
# define USBFRONT_IRQF 0
#else
# define USBFRONT_IRQF IRQF_SAMPLE_RANDOM
#endif

/* Port are numbered from 1 in linux */
#define vusb_device_by_port(v, port) (&(v)->vdev_ports[(port) - 1])
#define vusb_check_port(req, index)					\
do {								\
	if ((index) < 1 || (index) > VUSB_PORTS) {		\
		wprintk(req" with invalid port %u", (index));	\
		retval = -EPIPE;				\
		break;						\
	}							\
} while (0)

/* TODO RJP this stuff will go avay, it is v4v centric biz */
/*
 * Convenient alias to declare an iovec
 * @packet: name of the packet
 * @nchunk: number of chunk (ex: 2 => header + data)
 */
#define vusb_create_packet(name, nchunk) struct iovec name[(nchunk)]

/* Convenient alias to set header/data */
#define vusb_iov_set(packet, off, data, len)			\
	do {							\
		(packet)[(off)].iov_base = (data);		\
		(packet)[(off)].iov_len = (len);		\
	} while (0)

#define vusb_set_packet_header(packet, header, hlen)		\
	vusb_iov_set(packet, 0, header, hlen)

#define vusb_set_packet_data(packet, data, dlen)		\
	vusb_iov_set(packet, 1, data, dlen)

/* Possible state of an urbp */
enum vusb_urbp_state {
	VUSB_URBP_NEW,
	VUSB_URBP_SENT,
	VUSB_URBP_DONE,
	VUSB_URBP_DROP, /* Only here for debug purpose, it's same as done */
	VUSB_URBP_CANCEL,
};

/* URB tracking structure */
struct vusb_urbp {
	struct urb		*urb;
	enum vusb_urbp_state	state;
	u16                     handle;
	struct list_head	urbp_list;
	int                     port;
	usbif_request_t         request;
	u32			nr_segments;
};

struct vusb_vhcd;

/* Virtual USB device on of the RH ports */
struct vusb_device {
	u16			device_id;
	u32			port_status;
	u16			address;
	u16			port;
	enum usb_device_speed	speed;

	/* The Xenbus device associated with this vusb device */
	struct xenbus_device    *xendev;

	/* Pointer back to the parent virtual HCD core device */
	struct vusb_vhcd        *parent;

	/* This VUSB device's list of pending URBs */
	struct list_head        urbp_list;

	/* Xen rings and event channel */
	int			ring_ref;
	struct usbif_front_ring ring;
	unsigned int 		evtchn;
	unsigned int 		irq;

	unsigned		present:1;
	unsigned		connecting:1;
	unsigned		reset:2;
};

/* Virtual USB HCD/RH pieces */
enum vusb_rh_state {
	VUSB_RH_RESET,
	VUSB_RH_SUSPENDED,
	VUSB_RH_RUNNING
};

enum vusb_state {
	VUSB_INACTIVE,
	VUSB_WAIT_BIND_RESPONSE,
	VUSB_RUNNING,
};

struct vusb_vhcd {
	spinlock_t			lock;

	enum vusb_state			state;
	enum vusb_rh_state		rh_state;
	
	struct vusb_device		vdev_ports[VUSB_PORTS];
	/* TODO RJP this will go away */
	struct list_head                urbp_list;
	u16				urb_handle;

	/* Main thread */
	struct task_struct 		*kthread;

	/*
	 * Update hub can't be done in critical section.
	 * Is the driver need to update the hub?
	 */
	unsigned			poll:1;
};

static struct platform_device *vusb_platform_device = NULL;
static DEFINE_MUTEX(vusb_xen_pm_mutex);

static int
vusb_send_packet(struct vusb_vhcd *vhcd, const struct iovec *iovec, size_t niov);
static void
vusb_initialize_packet(struct vusb_vhcd *vhcd, void *packet, u16 devid,
		u8 command, u32 hlen, u32 dlen);
static void
vusb_send_reset_device_cmd(struct vusb_vhcd *vhcd, struct vusb_device *vdev);

/****************************************************************************/
/* Miscellaneous Routines                                                   */

static inline struct vusb_vhcd*
hcd_to_vhcd(struct usb_hcd *hcd)
{
	return (struct vusb_vhcd *)(hcd->hcd_priv);
}

static inline struct usb_hcd*
vhcd_to_hcd(struct vusb_vhcd *vhcd)
{
	return container_of((void *)vhcd, struct usb_hcd, hcd_priv);
}

static inline struct device*
vusb_dev(struct vusb_vhcd *vhcd)
{
	return vhcd_to_hcd(vhcd)->self.controller;
}

#ifdef VUSB_DEBUG

/* Convert Root Hub state in a string */
static const char*
vusb_rhstate_to_string(const struct vusb_vhcd *vhcd)
{
	switch (vhcd->rh_state) {
	case VUSB_RH_RESET:
		return "RESET";
	case VUSB_RH_SUSPENDED:
		return "SUSPENDED";
	case VUSB_RH_RUNNING:
		return "RUNNING";
	default:
		return "Unknown";
	}
}

/* Convert urb pipe type to string */
static const char *
vusb_pipe_to_string(struct urb *urb)
{

	switch (usb_pipetype(urb->pipe)) {
	case PIPE_ISOCHRONOUS:
		return "ISOCHRONOUS";
	case PIPE_CONTROL:
		return "CONTROL";
	case PIPE_INTERRUPT:
		return "INTERRUPT";
	case PIPE_BULK:
		return "BULK";
	default:
		return "Unknown";
	}
}

/* Convert urbp state to string */
static const char *
vusb_state_to_string(const struct vusb_urbp *urbp)
{
	switch (urbp->state) {
	case VUSB_URBP_NEW:
		return "NEW";
	case VUSB_URBP_SENT:
		return "SENT";
	case VUSB_URBP_DONE:
		return "DONE";
	case VUSB_URBP_DROP:
		return "DROP";
	case VUSB_URBP_CANCEL:
		return "CANCEL";
	default:
		return "unknown";
	}
}

#endif /* VUSB_DEBUG */

/*
 * Notify the worker that there is a new task
 * FIXME: I think the best solution is to have
 * a pending queue
 */
static inline void
vusb_worker_notify(struct vusb_vhcd *vhcd)
{
	send_sig(SIGINT, vhcd->kthread, 0);
}

/* Dump the URBp list */
static inline void
vusb_urbp_list_dump(const struct vusb_vhcd *vhcd, const char *fn)
{
	const struct vusb_urbp *urbp;

	dprintk(D_URB2, "===== Current URB List in %s =====\n", fn);
	list_for_each_entry(urbp, &vhcd->urbp_list, urbp_list) {
		dprintk(D_URB1, "URB handle 0x%x port %u device %u\n",
			urbp->handle, urbp->port,
			vusb_device_by_port(vhcd, urbp->port)->device_id);
	}
	dprintk(D_URB2, "===== End URB List in %s ====\n", fn);
}

#ifdef VUSB_DEBUG
/* Dump URBp */
static inline void
vusb_urbp_dump(struct vusb_urbp *urbp)
{
	struct urb *urb = urbp->urb;
	unsigned int type;

	type = usb_pipetype(urb->pipe);

	iprintk("urb handle: 0x%x state: %s status: %d pipe: %s(%u)\n",
		urbp->handle, vusb_state_to_string(urbp),
		urb->status, vusb_pipe_to_string(urb), type);
	iprintk("Device: %u Endpoint: %u In: %u\n",
		usb_pipedevice(urb->pipe),
		usb_pipeendpoint(urb->pipe),
		usb_urb_dir_in(urb));
}
#endif /* VUSB_DEBUG */

static inline u16
usb_speed_to_port_stat(enum usb_device_speed speed)
{
	switch (speed) {
	case USB_SPEED_HIGH:
		return USB_PORT_STAT_HIGH_SPEED;
	case USB_SPEED_LOW:
		return USB_PORT_STAT_LOW_SPEED;
	case USB_SPEED_FULL:
	default:
		return 0;
	}
}

static void
vusb_set_link_state(struct vusb_device *vdev)
{
	u32 newstatus, diff;

	newstatus = vdev->port_status;
	dprintk(D_STATE, "SLS: Port index %u status 0x%08x\n",
			vdev->port, newstatus);

	if (vdev->present) {
		newstatus |= (USB_PORT_STAT_CONNECTION) |
					usb_speed_to_port_stat(vdev->speed);
	} else {
		newstatus &= ~(USB_PORT_STAT_CONNECTION |
					USB_PORT_STAT_LOW_SPEED |
					USB_PORT_STAT_HIGH_SPEED |
					USB_PORT_STAT_ENABLE |
					USB_PORT_STAT_SUSPEND);
	}
	if ((newstatus & USB_PORT_STAT_POWER) == 0) {
		newstatus &= ~(USB_PORT_STAT_CONNECTION |
					USB_PORT_STAT_LOW_SPEED |
					USB_PORT_STAT_HIGH_SPEED |
					USB_PORT_STAT_SUSPEND);
	}
	diff = vdev->port_status ^ newstatus;

	if ((newstatus & USB_PORT_STAT_POWER) &&
	    (diff & USB_PORT_STAT_CONNECTION)) {
		newstatus |= (USB_PORT_STAT_C_CONNECTION << 16);
		dprintk(D_STATE, "Port %u connection state changed: %08x\n",
				vdev->port, newstatus);
	}

	vdev->port_status = newstatus;
}

/****************************************************************************/
/* VUSB HCD & RH                                                            */

/* SetFeaturePort(PORT_RESET) */
static void
vusb_port_reset(struct vusb_vhcd *vhcd, struct vusb_device *vdev)
{
	printk(KERN_DEBUG"vusb: port reset %u 0x%08x",
		   vdev->port, vdev->port_status);

	vdev->port_status |= USB_PORT_STAT_ENABLE | USB_PORT_STAT_POWER;

	vdev->reset = 1;

	vusb_worker_notify(vhcd);
}

static void
set_port_feature(struct vusb_vhcd *vhcd, struct vusb_device *vdev, u16 val)
{
	if (!vdev)
		return;

	switch (val) {
	case USB_PORT_FEAT_INDICATOR:
	case USB_PORT_FEAT_SUSPEND:
		/* Ignored now */
		break;

	case USB_PORT_FEAT_POWER:
		vdev->port_status |= USB_PORT_STAT_POWER;
		break;
	case USB_PORT_FEAT_RESET:
		vusb_port_reset(vhcd, vdev);
		break;
	case USB_PORT_FEAT_C_CONNECTION:
	case USB_PORT_FEAT_C_RESET:
	case USB_PORT_FEAT_C_ENABLE:
	case USB_PORT_FEAT_C_SUSPEND:
	case USB_PORT_FEAT_C_OVER_CURRENT:
		vdev->port_status &= ~(1 << val);
		break;

	default:
		/* No change needed */
		return;
	}
	vusb_set_link_state(vdev);
}

static void
clear_port_feature(struct vusb_device *vdev, u16 val)
{
	switch (val) {
	case USB_PORT_FEAT_INDICATOR:
	case USB_PORT_FEAT_SUSPEND:
		/* Ignored now */
		break;

	case USB_PORT_FEAT_ENABLE:
		vdev->port_status &= ~USB_PORT_STAT_ENABLE;
		vusb_set_link_state(vdev);
		break;

	case USB_PORT_FEAT_POWER:
		vdev->port_status &= ~(USB_PORT_STAT_POWER | USB_PORT_STAT_ENABLE);
		vusb_set_link_state(vdev);
		break;

	case USB_PORT_FEAT_C_CONNECTION:
	case USB_PORT_FEAT_C_RESET:
	case USB_PORT_FEAT_C_ENABLE:
	case USB_PORT_FEAT_C_SUSPEND:
	case USB_PORT_FEAT_C_OVER_CURRENT:
		dprintk(D_PORT1, "Clear bit %d, old 0x%08x mask 0x%08x new 0x%08x\n",
				val, vdev->port_status, ~(1 << val),
				vdev->port_status & ~(1 << val));
		vdev->port_status &= ~(1 << val);
		break;

	default:
		/* No change needed */
		return;
	}
}

/* Get a uniq URB handle */
static u16
vusb_get_urb_handle(struct vusb_vhcd *vhcd)
{
	vhcd->urb_handle += 1;

	if (vhcd->urb_handle >= 0xfff0)
		/* reset to 0 we never have lots URB in the list */
		vhcd->urb_handle = 0;

	return vhcd->urb_handle;
}

/*
 * Notify USB stack that the URB is finished and release it
 * The lock is already taken
 */
static void
vusb_urbp_release(struct vusb_vhcd *vhcd, struct vusb_urbp *urbp)
{
	struct urb *urb = urbp->urb;

#ifdef VUSB_DEBUG
	if (urb->status)
		vusb_urbp_dump(urbp);
#endif

	dprintk(D_URB1, "Giveback URB 0x%x status %d length %u\n",
		urbp->handle, urb->status, urb->actual_length);
	list_del(&urbp->urbp_list);
	kfree(urbp);
	usb_hcd_unlink_urb_from_ep(vhcd_to_hcd(vhcd), urb);
	/* Unlock the lock before notify the USB stack (could call other cb) */
	spin_unlock(&vhcd->lock);
	usb_hcd_giveback_urb(vhcd_to_hcd(vhcd), urb, urb->status);
	spin_lock(&vhcd->lock);
}

/* Retrieve device by device ID */
static struct vusb_device *
vusb_device_by_devid(struct vusb_vhcd *vhcd, u16 id)
{
	u16 i;

	for (i = 0; i < VUSB_PORTS; i++) {
		struct vusb_device *vdev = &vhcd->vdev_ports[i];
		if (vdev->present && vdev->device_id == id)
			return &vhcd->vdev_ports[i];
	}

	return NULL;
}

/* Hub descriptor */
static void
vusb_hub_descriptor(struct usb_hub_descriptor *desc)
{
	u16 temp;

	desc->bDescriptorType = 0x29;
	desc->bPwrOn2PwrGood = 10; /* echi 1.0, 2.3.9 says 20ms max */
	desc->bHubContrCurrent = 0;
	desc->bNbrPorts = VUSB_PORTS;

	/* size of DeviceRemovable and PortPwrCtrlMask fields */
	temp = 1 + (VUSB_PORTS / 8);
	desc->bDescLength = 7 + 2 * temp;

	/* bitmaps for DeviceRemovable and PortPwrCtrlMask */

#if ( LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,39) || (defined(RHEL_RELEASE_CODE)) )
	/* The union was introduced to support USB 3.0 */
	memset(&desc->u.hs.DeviceRemovable[0], 0, temp);
	memset(&desc->u.hs.DeviceRemovable[temp], 0xff, temp);
#else
	memset(&desc->DeviceRemovable[0], 0, temp);
	memset(&desc->DeviceRemovable[temp], 0xff, temp);
#endif

	/* per-port over current reporting and no power switching */
	temp = 0x00a;
	desc->wHubCharacteristics = cpu_to_le16(temp);
}

/* HCD start */
static int
vusb_hcd_start(struct usb_hcd *hcd)
{
	struct vusb_vhcd *vhcd = hcd_to_vhcd(hcd);

	iprintk("XEN HCD start\n");

	dprintk(D_MISC, ">vusb_start\n");

	vhcd->rh_state = VUSB_RH_RUNNING;

	hcd->power_budget = POWER_BUDGET;
	hcd->state = HC_STATE_RUNNING;
	hcd->uses_new_polling = 1;

	dprintk(D_MISC, "<vusb_start 0\n");

	return 0;
}

/* HCD stop */
static void vusb_hcd_stop(struct usb_hcd *hcd)
{
	struct vusb_vhcd *vhcd;

	iprintk("XEN HCD stop\n");

	dprintk(D_MISC, ">vusb_stop\n");

	vhcd = hcd_to_vhcd(hcd);

	hcd->state = HC_STATE_HALT;
	/* TODO: remove all URBs */

	//device_remove_file (dummy_dev(dum), &dev_attr_urbs);
	dev_info(vusb_dev(vhcd), "stopped\n");
	dprintk(D_MISC, "<vusb_stop\n");
}

static int
vusb_hcd_urb_enqueue(struct usb_hcd *hcd, struct urb *urb, gfp_t mem_flags)
{
	struct vusb_vhcd *vhcd;
	unsigned long flags;
	struct vusb_urbp *urbp;
	const struct vusb_device *vdev;

	int r = -ENOMEM;

	dprintk(D_MISC, ">vusb_urb_enqueue\n");

	vhcd = hcd_to_vhcd(hcd);

	if (!urb->transfer_buffer && urb->transfer_buffer_length)
		return -EINVAL;

	urbp = kmalloc(sizeof *urbp, mem_flags);
	if (!urbp)
		return -ENOMEM;

	urbp->state = VUSB_URBP_NEW;
	/* Port numbered from 1 */
	urbp->port = urb->dev->portnum;
	urbp->urb = urb;
	spin_lock_irqsave(&vhcd->lock, flags);
	vdev = vusb_device_by_port(vhcd, urbp->port);
	/* Allocate a handle */
	urbp->request.id = vusb_get_urb_handle(vhcd);

	if (vhcd->state == VUSB_INACTIVE || !vdev->present) {
		dprintk(D_WARN, "Worker is not up\n");
		kfree(urbp);
		r = -ESHUTDOWN;
		goto done;
	}

	r = usb_hcd_link_urb_to_ep(hcd, urb);
	if (r) {
		kfree(urbp);
		goto done;
	}

	list_add_tail(&urbp->urbp_list, &vhcd->urbp_list);
	vusb_worker_notify(vhcd);

done:
	spin_unlock_irqrestore(&vhcd->lock, flags);

	return r;
}

static int
vusb_hcd_urb_dequeue(struct usb_hcd *hcd, struct urb *urb, int status)
{
	struct vusb_vhcd *vhcd;
	unsigned long flags;
	int rc;
	struct vusb_urbp *urbp;

	dprintk(D_MISC, "*vusb_urb_dequeue\n");
	vhcd = hcd_to_vhcd(hcd);

	spin_lock_irqsave(&vhcd->lock, flags);

	rc = usb_hcd_check_unlink_urb(hcd, urb, status);

	if (rc)
		goto out_dequeue;

	urb->status = status;

	/* Retrieve URBp */
	list_for_each_entry(urbp, &vhcd->urbp_list, urbp_list) {
		if (urbp->urb == urb)
			break;
	}

	if (urbp) {
		urbp->state = VUSB_URBP_CANCEL;
		vusb_worker_notify(vhcd);
	} else
		wprintk("Try do dequeue an unhandle URB\n");

out_dequeue:
	spin_unlock_irqrestore(&vhcd->lock, flags);

	return rc;
}

static int
vusb_hcd_get_frame(struct usb_hcd *hcd)
{
	struct timeval	tv;

	dprintk(D_MISC, "*vusb_get_frame\n");
	do_gettimeofday(&tv);

	return tv.tv_usec / 1000;
}

#define PORT_C_MASK \
	((USB_PORT_STAT_C_CONNECTION \
	| USB_PORT_STAT_C_ENABLE \
	| USB_PORT_STAT_C_SUSPEND \
	| USB_PORT_STAT_C_OVERCURRENT \
	| USB_PORT_STAT_C_RESET) << 16)

static int
vusb_hcd_hub_status(struct usb_hcd *hcd, char *buf)
{
	struct vusb_vhcd *vhcd = hcd_to_vhcd(hcd);
	unsigned long flags;
        int resume = 0;
	int changed = 0;
	u16 length = 0;
	int ret = 0;
	u16 i;

	dprintk(D_MISC, ">vusb_hub_status\n");

	/* FIXME: Not sure it's good */
	if (!HCD_HW_ACCESSIBLE(hcd)) {
		wprintk("Hub is not running %u\n", hcd->state);
		dprintk(D_MISC, ">vusb_hub_status 0\n");
		return 0;
	}

	/* Initialize the status to no-change */
	length = 1 + (VUSB_PORTS / 8);
	for (i = 0; i < length; i++)
		buf[i] = 0;

	spin_lock_irqsave(&vhcd->lock, flags);

	for (i = 0; i < VUSB_PORTS; i++) {
		struct vusb_device *vdev = &vhcd->vdev_ports[i];

		/* Check status for each port */
		dprintk(D_PORT2, "check port %u (%08x)\n", vhcd->vdev_ports[i].port,
				vhcd->device[i].port_status);
		if ((vdev->port_status & PORT_C_MASK) != 0) {
			if (i < 7)
				buf[0] |= 1 << (i + 1);
			else if (i < 15)
				buf[1] |= 1 << (i - 7);
			else if (i < 23)
				buf[2] |= 1 << (i - 15);
			else
				buf[3] |= 1 << (i - 23);
			dprintk(D_PORT2, "port %u status 0x%08x has changed\n",
				    vdev->port, vdev->port_status);
			changed = 1;
		}

                if (vdev->port_status & USB_PORT_STAT_CONNECTION)
                        resume = 1;
	}

	if (resume && vhcd->rh_state == VUSB_RH_SUSPENDED)
		usb_hcd_resume_root_hub(hcd);

	ret = (changed) ? length : 0;

	spin_unlock_irqrestore(&vhcd->lock, flags);
	dprintk(D_MISC, "<vusb_hub_status %d\n", ret);

	return ret;
}

static int
vusb_hcd_hub_control(struct usb_hcd *hcd, u16 typeReq, u16 wValue,
		u16 wIndex, char *buf, u16 wLength)
{
	struct vusb_vhcd *vhcd;
	int	retval = 0;
	unsigned long flags;
	u32 status;

	dprintk(D_CTRL, ">vusb_hub_control %04x %04x %04x\n",
			typeReq, wIndex, wValue);

	if (!HCD_HW_ACCESSIBLE(hcd)) {
		dprintk(D_CTRL, "<vusb_hub_control %d\n", ETIMEDOUT);
		return -ETIMEDOUT;
	}

	vhcd = hcd_to_vhcd(hcd);
	spin_lock_irqsave(&vhcd->lock, flags);
	switch (typeReq) {
	case ClearHubFeature:
		break;
	case ClearPortFeature:
		dprintk(D_CTRL, "ClearPortFeature port %d val: 0x%04x\n",
				wIndex, wValue);
		vusb_check_port("ClearPortFeature", wIndex);
	    	clear_port_feature(vusb_device_by_port(vhcd, wIndex), wValue);
		break;
	case GetHubDescriptor:
		vusb_hub_descriptor((struct usb_hub_descriptor *)buf);
		break;
	case GetHubStatus:
		/* Always local power supply good and no over-current exists. */
		*(__le32 *)buf = cpu_to_le32(0);
		break;
	case GetPortStatus:
		vusb_check_port("GetPortStatus", wIndex);
		status = vusb_device_by_port(vhcd, wIndex)->port_status;
		status = vhcd->vdev_ports[wIndex-1].port_status;
		dprintk(D_CTRL, "GetPortStatus port %d = 0x%08x\n", wIndex, status);
		((__le16 *) buf)[0] = cpu_to_le16(status);
		((__le16 *) buf)[1] = cpu_to_le16(status >> 16);
		break;
	case SetHubFeature:
		retval = -EPIPE;
		break;
	case SetPortFeature:
		vusb_check_port("SetPortStatus", wIndex);
		dprintk(D_CTRL, "SetPortFeature port %d val: 0x%04x\n", wIndex, wValue);
		set_port_feature(vhcd, vusb_device_by_port(vhcd, wIndex), wValue);
		break;

	default:
		dev_dbg(vusb_dev(vhcd),
			"hub control req%04x v%04x i%04x l%d\n",
			typeReq, wValue, wIndex, wLength);

		/* "protocol stall" on error */
		retval = -EPIPE;
	}
	spin_unlock_irqrestore(&vhcd->lock, flags);

	if (wIndex >= 1 && wIndex <= VUSB_PORTS) {
		if ((vusb_device_by_port(vhcd, wIndex)->port_status & PORT_C_MASK) != 0)
			 usb_hcd_poll_rh_status (hcd);
	}

	dprintk(D_MISC, "<vusb_hub_control %d\n", retval);
	return retval;
}

#ifdef CONFIG_PM
static int
vusb_hcd_bus_suspend(struct usb_hcd *hcd)
{
	struct vusb_vhcd *vhcd = hcd_to_vhcd(hcd);
	unsigned long flags;

	dprintk(D_PM, "Bus suspend\n");

	spin_lock_irqsave(&vhcd->lock, flags);
	vhcd->rh_state = VUSB_RH_SUSPENDED;
	spin_unlock_irqrestore(&vhcd->lock, flags);

	return 0;
}

static int
vusb_hcd_bus_resume(struct usb_hcd *hcd)
{
	struct vusb_vhcd *vhcd = hcd_to_vhcd(hcd);
	int rc = 0;

	dprintk(D_PM, "Bus resume\n");

	spin_lock_irq(&vhcd->lock);
	if (!HCD_HW_ACCESSIBLE(hcd)) {
		rc = -ESHUTDOWN;
	} else {
		vhcd->rh_state = VUSB_RH_RUNNING;
		hcd->state = HC_STATE_RUNNING;
	}
	spin_unlock_irq(&vhcd->lock);
	return rc;
}
#endif /* CONFIG_PM */


static const struct hc_driver vusb_hcd_driver = {
	.description = VUSB_HCD_DRIVER_NAME,
	.product_desc =	VUSB_DRIVER_DESC,
	.hcd_priv_size = sizeof(struct vusb_vhcd),

	.flags = HCD_USB2,

	/* .reset not used since our HCD is so simple, everything is done in start */
	.start = vusb_hcd_start,
	.stop =	vusb_hcd_stop,

	.urb_enqueue = vusb_hcd_urb_enqueue,
	.urb_dequeue = vusb_hcd_urb_dequeue,

	.get_frame_number = vusb_hcd_get_frame,

	.hub_status_data = vusb_hcd_hub_status,
	.hub_control = vusb_hcd_hub_control,
#ifdef CONFIG_PM
	.bus_suspend = vusb_hcd_bus_suspend,
	.bus_resume = vusb_hcd_bus_resume,
#endif /* CONFIG_PM */
};

/****************************************************************************/
/* URB Processing                                                           */

/* Retrieve a URB by handle */
static struct vusb_urbp*
vusb_urb_by_handle(struct vusb_vhcd *vhcd, struct vusb_device *vdev, u16 handle)
{
	struct vusb_urbp *urbp;

	list_for_each_entry(urbp, &vhcd->urbp_list, urbp_list) {
		/*
		 * Check both handle and port to avoid to use an URB
		 * of another device
		 */
		if (urbp->handle == handle && urbp->port == vdev->port)
			return urbp;
	}

	dprintk(D_URB1, "Unable to retrieve URB handle 0x%x port %u\n",
		handle, vdev->port);
	vusb_urbp_list_dump(vhcd, __FUNCTION__);

	return NULL;
}

/* Common part to finish an URB request */
static void
vusb_urb_common_finish(struct vusb_vhcd *vhcd, struct vusb_urbp *urbp, bool in,
			u32 len, const u8 *data)
{
	struct urb *urb = urbp->urb;

	if (!in) { /* Outbound */
		dprintk(D_URB2, "Outgoing URB completed status %d\n",
			urb->status);
		/* Sanity check on len, should be 0 */
		if (len) {
			wprintk("Data not expected for outgoing URB\n");
			urb->status = -EIO;
		} else {
			/*
			 * FIXME: move this part in send
			 * For outgoing URB, the actual length is the length
			 * transfered to the vusb daemon
			 */
			urb->actual_length = urb->transfer_buffer_length;
		}
	} else { /* Inbound */
		dprintk(D_URB2, "Incoming URB completed status %d len %u\n",
			urb->status, len);
		/*
		 * Sanity check on len, should be less or equal to
		 * the length of the transfer buffer
		 */
		if (len > urb->transfer_buffer_length) {
			wprintk("Length mismatch for incoming URB"
				" (wanted %u bug got %u)\n",
				urb->transfer_buffer_length, len);
			urb->status = -EIO;
		} else {
			dprintk(D_URB2, "In %u bytes out of %u\n",
				len, urb->transfer_buffer_length);

			urb->actual_length = len;
			/* FIXME: use transfer buffer directly to read */
			if (len > 0)
				memcpy(urb->transfer_buffer, data, len);
		}
	}

	vusb_urbp_release(vhcd, urbp);
}

/*
 * Finish an isochronous URB
 */
static void
vusb_urb_isochronous_finish(struct vusb_vhcd *vhcd, struct vusb_urbp *urbp,
			u32 len, const u8 *data)
{
	struct urb *urb = urbp->urb;
	u32 hlen = 0 /* STUB get header lenght */;
	u32 dlen = 0;
	int i;

	/* STUB sanity check ISO URB */

	/* STUB if data is not the response, move ptr */

	for (i = 0; i < urb->number_of_packets; i++) {
		struct usb_iso_packet_descriptor *desc = &urb->iso_frame_desc[i];
		u32 plen = 0 /* STUB ISO response lenght */;

		/* Sanity check on packet length */
		if (plen > desc->length) {
			wprintk("iso packet %d too much data\n", i);
			goto iso_err;
		}

		desc->actual_length = plen;
		desc->status = 0 /* STUB ISO status */;

		if (usb_urb_dir_in(urb)) {
			/* Do sanity check each time on effective data length */
			if (len < (hlen + dlen + plen)) {
				wprintk("Short URB Iso Response Data."
					"Expected %u got %u\n",
					dlen + plen, len - hlen);
				goto iso_err;
			}
			/* Copy to the right offset */
			memcpy(&(((u8 *)urb->transfer_buffer)[desc->offset]),
				&data[dlen], plen);
		}
		dlen += plen;
	}

	urb->actual_length = dlen;

	vusb_urbp_release(vhcd, urbp);
	return;

iso_err:
	urb->status = -EIO;
	for (i = 0; i < urb->number_of_packets; i++) {
		urb->iso_frame_desc[i].actual_length = 0;
		urb->iso_frame_desc[i].status = urb->status;
	}
	urb->actual_length = 0;

	vusb_urbp_release(vhcd, urbp);
}

/* Finish a control URB */
static void
vusb_urb_control_finish(struct vusb_vhcd *vhcd, struct vusb_urbp *urbp, u32 len, const u8 *data)
{
	const struct usb_ctrlrequest *ctrl;
	bool in;

	ctrl = (struct usb_ctrlrequest *)urbp->urb->setup_packet;

	in = (ctrl->bRequestType & USB_DIR_IN) != 0;

	vusb_urb_common_finish(vhcd, urbp, in, len, data);
}

/* Finish a bulk URB */
static void
vusb_urb_bulk_finish(struct vusb_vhcd *vhcd, struct vusb_urbp *urbp, u32 len, const u8 *data)
{
	vusb_urb_common_finish(vhcd, urbp, usb_urb_dir_in(urbp->urb), len, data);
}

/* Finish an interrupt URB */
static void
vusb_urb_interrupt_finish(struct vusb_vhcd *vhcd, struct vusb_urbp *urbp, u32 len, const u8 *data)
{
	vusb_urb_common_finish(vhcd, urbp, usb_urb_dir_in(urbp->urb), len, data);
}

/* Convert status to errno */
static int
vusb_urb_status_to_errno(u32 status)
{
	int32_t st = status;

	switch (status) {
	case VUSB_URB_STATUS_SUCCESS:
		return 0;
	/* STUB probably want others */
	case VUSB_URB_STATUS_FAILURE:
		return -EIO;
	default:
		if (st < 0) /* Already an errno */
			return st;
		else
			return -EIO;
	}
}

#ifdef VUSB_DEBUG
/* Convert status to a string */
static const char*
vusb_status_to_string(u32 status)
{
	int32_t st = status;

	switch (status) {
	case VUSB_URB_STATUS_SUCCESS:
		return "SUCCESS";
	/* STUB probably want others */
	case VUSB_URB_STATUS_FAILURE:
		return "FAILURE";
	default:
		if (st < 0) /* Already an errno */
			return "ERRNO";
		else
			return "UNKNOWN";
	}
}
#endif /* VUSB_DEBUG */

/*
 * Finish an URB request
 * @packet: used by isochronous URB because we need the header FIXME
 */
static void
vusb_urb_finish(struct vusb_vhcd *vhcd, struct vusb_device *vdev, u16 handle,
		u32 status, u32 len, const u8 *data)
{
	struct vusb_urbp *urbp;
	struct urb *urb;

	urbp = vusb_urb_by_handle(vhcd, vdev, handle);

	if (!urbp) {
		dprintk(D_WARN, "Bad handle (0x%x) for Device ID (%u)\n",
			handle, vdev->device_id);
		return;
	}

	urb = urbp->urb;
	urb->status = vusb_urb_status_to_errno(status);

	switch (usb_pipetype(urb->pipe)) {
	case PIPE_ISOCHRONOUS:
		vusb_urb_isochronous_finish(vhcd, urbp, len, data);
		break;

	case PIPE_CONTROL:
		vusb_urb_control_finish(vhcd, urbp, len, data);
		break;

	case PIPE_INTERRUPT:
		vusb_urb_interrupt_finish(vhcd, urbp, len, data);
		break;

	case PIPE_BULK:
		vusb_urb_bulk_finish(vhcd, urbp, len, data);
		break;

	default:
		wprintk("Unknow pipe type %u\n",
			usb_pipetype(urb->pipe));
	}
}

/* Handle command URB response */
static void
vusb_handle_urb_response(struct vusb_vhcd *vhcd, const void *packet)
{
	unsigned long flags;
	struct vusb_device *vdev;
	u16 handle = 0 /* STUB logical handle */;
	u32 status = 0;
	u32 len = 0;

	/* STUB sanity check and get response values */

	spin_lock_irqsave(&vhcd->lock, flags);

	vdev = vusb_device_by_devid(vhcd, 0 /* STUB logical device ID */);
	if (!vdev) {
		wprintk("Bad device ID (%u) in URB response\n", 0);
		goto out;
	}

	vusb_urb_finish(vhcd, vdev, handle, status, len, NULL /* STUB the response data */);
out:
	spin_unlock_irqrestore(&vhcd->lock, flags);
}

/* Handle command URB status */
static void
vusb_handle_urb_status(struct vusb_vhcd *vhcd, const void *packet)
{
	unsigned long flags;
	struct vusb_device *vdev;
	u16 handle = 0 /* STUB logical handle */;
	u32 status = 0;

	/* STUB sanity check and get status values */

	spin_lock_irqsave(&vhcd->lock, flags);

	vdev = vusb_device_by_devid(vhcd, 0 /* STUB logical device ID */);
	if (!vdev) {
		wprintk("Bad device ID (%u) in URB Status\n", 0);
		goto out;
	}

	vusb_urb_finish(vhcd, vdev, handle, status, 0, NULL);
out:
	spin_unlock_irqrestore(&vhcd->lock, flags);
}

/*
 * Initialize an URB packet
 * @packet: packet to initialize (already allocated)
 * @command: what do we want?
 * @hlen: size of header
 * @has_data: if true, length will be the one of the transfer buffer
 */
static void
vusb_initialize_urb_packet(struct vusb_vhcd *vhcd, void *packet,
		const struct vusb_urbp *urbp, struct vusb_device *vdev,
		u8 command, u32 hlen, bool has_data)
{
	if (has_data) /* Outbound request */
		vusb_initialize_packet(vhcd, packet, vdev->device_id,
				command, hlen, urbp->urb->transfer_buffer_length);
	else
		vusb_initialize_packet(vhcd, packet, vdev->device_id,
				command, hlen, 0);

	/* STUB get logical handle: urbp->handle */
}

/*
 * Send an URB packet to the host
 * This function will setup the iov and add data if needed with the transfer
 * buffer
 * Doesn't fit for isochronous request
 */
static int
vusb_send_urb_packet(struct vusb_vhcd *vhcd, struct vusb_urbp *urbp, 
		void *packet, u32 hlen, bool has_data)
{
	vusb_create_packet(iovec, 2);
	int r;

	vusb_set_packet_header(iovec, packet, hlen);
	if (has_data)
		vusb_set_packet_data(iovec, urbp->urb->transfer_buffer,
				urbp->urb->transfer_buffer_length);

	r = vusb_send_packet(vhcd, iovec, (has_data) ? 2 : 1);

	if (r < 0) {
		/* An error occured drop the URB and notify the USB stack */
		urbp->state = VUSB_URBP_DROP;
		urbp->urb->status = r;
	} else
		urbp->state = VUSB_URBP_SENT;

	return r;
}

/* Convert URB transfer flags to VUSB flags */
static inline u16
vusb_urb_to_flags(struct urb *urb)
{
	u16 flags = 0;

	if (usb_urb_dir_in(urb))
		flags |= VUSB_URB_DIRECTION_IN;

	if (!(urb->transfer_flags & URB_SHORT_NOT_OK))
		flags |= VUSB_URB_SHORT_OK;

	if (urb->transfer_flags & URB_ISO_ASAP)
		flags |= VUSB_URB_ISO_TRANSFER_ASAP;

	return flags;
}

/* Retrieve endpoint from URB */
static inline u8
vusb_urb_to_endpoint(struct urb *urb)
{
	u8 endpoint = 0;

	endpoint = usb_pipeendpoint(urb->pipe);
	if (usb_urb_dir_in(urb))
		endpoint |= 0x80;

	return endpoint;
}

/* Not defined by hcd.h */
#define InterfaceOutRequest 						\
	((USB_DIR_OUT|USB_TYPE_STANDARD|USB_RECIP_INTERFACE) << 8)

/* Send an urb control to the host */
static void
vusb_send_control_urb(struct vusb_vhcd *vhcd, struct vusb_urbp *urbp)
{
	struct urb *urb = urbp->urb;
	struct vusb_device *vdev;
	void *packet;
	u32 hlen;
	const struct usb_ctrlrequest *ctrl;
	u8 bRequestType, bRequest;
	u16 typeReq, wValue, wIndex, wLength;
	bool in;
	bool has_data = 0;

	/* Convenient aliases on setup packet*/
	ctrl = (struct usb_ctrlrequest *)urb->setup_packet;
	bRequestType = ctrl->bRequestType;
	bRequest = ctrl->bRequest;
	wValue = le16_to_cpu(ctrl->wValue);
	wIndex = le16_to_cpu(ctrl->wIndex);
	wLength = le16_to_cpu(ctrl->wLength);

	typeReq = (bRequestType << 8) | bRequest;
	in = (bRequestType & USB_DIR_IN) != 0;

	dprintk(D_URB2,
		"Send Control URB Device: %u Endpoint: %u In: %u Cmd: 0x%x 0x%02x\n",
		usb_pipedevice(urb->pipe),
		usb_pipeendpoint(urb->pipe),
		in, ctrl->bRequest, ctrl->bRequestType);

	dprintk(D_URB2, "Setup packet, tb_len=%d\n", urb->transfer_buffer_length);
	dprint_hex_dump(D_URB2, "SET: ", DUMP_PREFIX_OFFSET, 16, 1, ctrl, 8, true);

	/* Retrieve the device */
	vdev = vusb_device_by_port(vhcd, urbp->port);

	switch (typeReq) {
	case DeviceOutRequest | USB_REQ_SET_ADDRESS:
		/* Don't forward set address command, directly return */
		vdev->address = wValue;
		dprintk(D_URB2, "SET ADDRESS %u\n", vdev->address);
		urb->status = 0;
		urbp->state = VUSB_URBP_DONE;
		return;

	case DeviceOutRequest | USB_REQ_SET_CONFIGURATION:
		hlen = 0 /* STUB set configuration length */;
		vusb_initialize_urb_packet(vhcd, &packet, urbp, vdev,
				0 /* STUB set configuration internal command */,
				hlen, false);

		/* STUB finish packet setup */
		break;

	case InterfaceOutRequest | USB_REQ_SET_INTERFACE:
		hlen = 0 /* STUB select interface length */;
		vusb_initialize_urb_packet(vhcd, &packet, urbp, vdev,
				0 /* STUB select interface  internal command */,
				hlen, false);

		/* STUB finish packet setup */
		break;

	default:
		hlen = 0 /* STUB control length */;
		vusb_initialize_urb_packet(vhcd, &packet, urbp, vdev,
				0 /* STUB control internal command */,
				hlen, !in);

		/* STUB finish packet setup */
	}

	vusb_send_urb_packet(vhcd, urbp, &packet, hlen, has_data /* STUB may or may not have data in packet */);
}

/* Send an URB interrup command */
static void
vusb_send_interrupt_urb(struct vusb_vhcd *vhcd, struct vusb_urbp *urbp)
{
	struct urb *urb = urbp->urb;
	struct vusb_device *vdev;
	void *packet;

	dprintk(D_URB2, "Send Interrupt URB Device: %u Endpoint: %u in: %u\n",
		usb_pipedevice(urb->pipe),
		usb_pipeendpoint(urb->pipe),
		usb_urb_dir_in(urb));

	vdev = vusb_device_by_port(vhcd, urbp->port);

	vusb_initialize_urb_packet(vhcd, &packet, urbp, vdev,
			0 /* STUB interrupt internal command */,
			0 /* STUB interrupt length */,
			usb_urb_dir_out(urb));

	/* STUB finish packet setup */

	vusb_send_urb_packet(vhcd, urbp, &packet,
			0 /* STUB interrupt length */,
			usb_urb_dir_out(urb));
}

/* Send an URB bulk command */
static void
vusb_send_bulk_urb(struct vusb_vhcd *vhcd, struct vusb_urbp *urbp)
{
	struct urb *urb = urbp->urb;
	struct vusb_device *vdev;
	void *packet;

	dprintk(D_URB2, "Send Bulk URB Device: %u Endpoint: %u in: %u\n",
		usb_pipedevice(urb->pipe),
		usb_pipeendpoint(urb->pipe),
		usb_urb_dir_in(urb));

	vdev = vusb_device_by_port(vhcd, urbp->port);

	vusb_initialize_urb_packet(vhcd, &packet, urbp, vdev,
			0 /* STUB bulk internal command */,
			0 /* STUB bulk length */,
			usb_urb_dir_out(urb));

	/* STUB finish packet setup */

	vusb_send_urb_packet(vhcd, urbp, &packet,
			0 /* STUB bulk length */,
			usb_urb_dir_out(urb));
}

/* Send an isochronous urb command */
static void
vusb_send_isochronous_urb(struct vusb_vhcd *vhcd, struct vusb_urbp *urbp)
{
	struct urb *urb = urbp->urb;
	struct vusb_device *vdev;
	void *packet;
	vusb_create_packet(iovec, 3);
	/* TODO: use typeof? */
	u32 length[urb->number_of_packets]; /* avoid kmalloc */
	int i = 0;
	int r;

	dprintk(D_URB2, "Send Isochronous URB Device: %u Endpoint: %u in: %u\n",
		usb_pipedevice(urb->pipe),
		usb_pipeendpoint(urb->pipe),
		usb_urb_dir_in(urb));

	dprintk(D_URB2, "Number of packets = %u\n", urb->number_of_packets);

	vdev = vusb_device_by_port(vhcd, urbp->port);

	/* Use the common urb initialization packet but fix ByteCount */
	vusb_initialize_urb_packet(vhcd, &packet, urbp, vdev,
			0 /* STUB isoch internal command */,
			0 /* STUB isoch length */,
			usb_urb_dir_out(urb));

	/* STUB finish packet setup */

	for (i = 0; i < urb->number_of_packets; i++)
	{
		dprintk(D_URB2, "packet %d offset = 0x%u length = 0x%u\n",
			i, urb->iso_frame_desc[i].offset,
			urb->iso_frame_desc[i].length);
		length[i] = urb->iso_frame_desc[i].length;
	}

	vusb_iov_set(iovec, 0, &packet, 0 /* STUB isoch length */);
	vusb_iov_set(iovec, 1, length, 0 /* STUB isoch packet length */
			* urb->number_of_packets);
	if (usb_urb_dir_out(urb))
		vusb_iov_set(iovec, 2, urb->transfer_buffer,
				urb->transfer_buffer_length);

	r = vusb_send_packet(vhcd, iovec, (usb_urb_dir_out(urb)) ? 3 : 2);

	if (r < 0) {
		/* An error occured drop the URB and notify the USB stack */
		urbp->state = VUSB_URBP_DROP;
		urbp->urb->status = r;
	} else
		urbp->state = VUSB_URBP_SENT;
}

/* Send a cancel URB command */
static void
vusb_send_cancel_urb(struct vusb_vhcd *vhcd, struct vusb_device *vdev,
		struct vusb_urbp *urbp)
{
	vusb_create_packet(iovec, 1);
	void *packet;

	vusb_initialize_packet(vhcd, &packet, vdev->device_id,
			0 /* STUB cancel internal command */,
			0 /* STUB cancel length */,
			0);

	/* STUB finish packet setup */

	dprintk(D_URB1, "send packet URB_CANCEL device %u port %u handle 0x%04x\n",
		vdev->device_id, vdev->port, urbp->handle);

	vusb_set_packet_header(iovec, &packet, 0 /* STUB cancel length */);
	vusb_send_packet(vhcd, iovec, 1);
}

/* Send an URB */
static void
vusb_send_urb(struct vusb_vhcd *vhcd, struct vusb_urbp *urbp)
{
	struct urb *urb = urbp->urb;
	struct vusb_device *vdev;
	unsigned int type;

	type = usb_pipetype(urb->pipe);

	dprintk(D_URB2, "urb handle: 0x%x status: %s pipe: %s(%u)\n",
		urbp->handle, vusb_state_to_string(urbp),
		vusb_pipe_to_string(urb), type);

	vdev = vusb_device_by_port(vhcd, urbp->port);

	if (urbp->state == VUSB_URBP_NEW) {
		switch (type) {
		case PIPE_ISOCHRONOUS:
			vusb_send_isochronous_urb(vhcd, urbp);
			break;

		case PIPE_INTERRUPT:
			vusb_send_interrupt_urb(vhcd, urbp);
			break;

		case PIPE_CONTROL:
			vusb_send_control_urb(vhcd, urbp);
			break;

		case PIPE_BULK:
			vusb_send_bulk_urb(vhcd, urbp);
			break;

		default:
			wprintk("Unknown urb type %x\n", type);
		}
	} else if (urbp->state == VUSB_URBP_CANCEL) {
		vusb_send_cancel_urb(vhcd, vdev, urbp);
	}

	if (urbp->state == VUSB_URBP_DONE ||
	    urbp->state == VUSB_URBP_DROP ||
	    urbp->state == VUSB_URBP_CANCEL) {
		/* Remove URB */
		dprintk(D_URB1, "URB immediate %s\n",
			vusb_state_to_string(urbp));
		vusb_urbp_release(vhcd, urbp);
	}
}

/*
 * Process URB task
 * - Check if we need to reset a device
 * - Browse and send URB
 */
static void
vusb_process_urbs(struct vusb_vhcd *vhcd)
{
	struct vusb_urbp *urbp;
	struct vusb_urbp *next;
	unsigned long flags;
	u16 i;

	dprintk(D_MISC, "process_urbs()\n");

	spin_lock_irqsave(&vhcd->lock, flags);

	/* Check if we need to reset a device */
	for (i = 0; i < VUSB_PORTS; i++) {
		if (vhcd->vdev_ports[i].reset == 1) {
			vhcd->vdev_ports[i].reset = 2;
			vusb_send_reset_device_cmd(vhcd, &vhcd->vdev_ports[i]);
		}
	}

	/* Browse URB list */
	list_for_each_entry_safe(urbp, next, &vhcd->urbp_list, urbp_list) {
		vusb_send_urb(vhcd, urbp);
	}

	spin_unlock_irqrestore(&vhcd->lock, flags);

	if (vhcd->poll) { /* Update Hub status */
		vhcd->poll = 0;
		usb_hcd_poll_rh_status(vhcd_to_hcd(vhcd));
	}
}

/****************************************************************************/
/* Daemon Thread                                                            */

/* Helper to cleanup data associated to the worker */
static void
vusb_worker_cleanup(struct vusb_vhcd *vhcd)
{
	struct vusb_urbp *urbp;
	struct vusb_urbp *next;
	u16 i = 0;
	unsigned long flags;

	dprintk(D_PM, "Clean up the worker\n");

	spin_lock_irqsave(&vhcd->lock, flags);
	vhcd->rh_state = VUSB_INACTIVE;

	list_for_each_entry_safe(urbp, next, &vhcd->urbp_list, urbp_list) {
		struct vusb_device *vdev;

		vdev = vusb_device_by_port(vhcd, urbp->port);
		urbp->urb->status = -ESHUTDOWN;
		vusb_urbp_release(vhcd, urbp);
	}

	/* Unplug all USB devices */
	for (i = 0; i < VUSB_PORTS; i++) {
		vhcd->vdev_ports[i].port = i + 1;
		vhcd->vdev_ports[i].connecting = 0;
		vhcd->vdev_ports[i].present = 0;
	}

	spin_unlock_irqrestore(&vhcd->lock, flags);
}

/*
 * Main task
 * - Read command
 * - Send command if the task receive an interrupt (not efficient)
 */
static void
vusb_mainloop(struct vusb_vhcd *vhcd)
{
	/* TODO RJP:
	 * process writes to rings that were full.
	 * process reads from rings that were empty.
	 *
	 * Original code for reference for now
	 *
	int nr = 0;
	int expected = 0; // STUB packet header length
	int count = 0;
	int r;
	// STUB get packet header

	// FIXME: check return 
	vusb_send_bind_request(v);
	do {
		nr = vusb_read(v, pbuf + count, expected - count);

		dprintk(D_V4V1, "vusb_read: %d\n", nr);

		if (nr == -EINTR || nr == -ERESTARTSYS) { // Sig INT occured 
			// Check if we need to stop
			if (kthread_should_stop())
			        return;
			flush_signals(current);
			vusb_process_urbs(v);
			dprintk(D_V4V1, "vusb: got interrupted, restarting read\n");
			continue;
		} else if (nr < 0) { // TODO: handle EAGAIN EDISCONNECT
			wprintk("Unexpected error on read: %d\n", nr);
			return;
		} else if (nr == 0) {
			wprintk("zero read, assuming server close connection\n");
			// TODO: Don't close the thread. Check if we can restart the connection 
			return;
		}

		count = count + nr;

		if (count < expected) {
			dprintk(D_V4V2, "Partial read, remaining: %d\n", expected-count);
			continue;
		} else  if (expected == 0) { // STUB packet header length 
			expected = 0; // STUB payload length 
			if (expected > VUSB_MAX_PACKET_SIZE) {
				wprintk("Packet too large (%u)\n", expected);
				// TODO: Skip the packet, don't close the connection
				return;
			}
		}

		if (count > expected) {
			BUG();
		}
		if (count == expected) {
			dprintk(D_V4V1, "All data received calling handler\n");
			r = vusb_process_packet(v, (void *)pbuf);
			if (v->poll) { // Update Hub status
				v->poll = 0;
				usb_hcd_poll_rh_status(vhcd_to_hcd(v));
			}

			if (r < 0) {
				return;
			}
			if (r == 2) {
				vusb_process_urbs(v);
			}
			expected = 0; // STUB packet header length
			count = 0;
		}
	} while(1);
	*/
}

static int
vusb_threadfunc(void *data)
{
	mm_segment_t oldfs;
	struct vusb_vhcd *vhcd = data;

	dprintk(D_VUSB1, "tf: In thread\n");

	/* Fine now, as we don't return to userspace: */
	oldfs = get_fs();
	set_fs(get_ds());

	siginitsetinv(&current->blocked, sigmask(SIGINT));
	allow_signal(SIGINT);

	/* Main loop */
	set_current_state(TASK_INTERRUPTIBLE);
	vusb_mainloop(vhcd);

	dprintk(D_VUSB1, "tf: fp closed, thread going idle\n");

	if (!kthread_should_stop())
		wprintk("Unexpected TODO close\n");

	vusb_worker_cleanup(vhcd);

	set_fs(oldfs);
	while (!kthread_should_stop()) {
		schedule_timeout(100000);
	}
	dprintk(D_VUSB1, "tf: Thread exiting\n");
	return 0;
}

/* Helper to create the worker */
static int
vusb_worker_start(struct vusb_vhcd *vhcd)
{
	int ret = 0;
	u16 i = 0;

	dprintk(D_PM, "Start the worker\n");

	/* TODO RJP revisit, may not want to clear the devices on resume path
	 * also may need suspend/resume for S3 */
	/* Initialize ports */
	for (i = 0; i < VUSB_PORTS; i++) {
		vhcd->vdev_ports[i].port = i + 1;
		vhcd->vdev_ports[i].parent = vhcd;
		vhcd->vdev_ports[i].connecting = 0;
		vhcd->vdev_ports[i].present = 0;
	}

	vhcd->rh_state = VUSB_INACTIVE;

	/* TODO Opened v4v connection here - the vusb devices will do this for their rings*/

	/* Create the main thread */
	vhcd->kthread = kthread_run(vusb_threadfunc, vhcd, "vusb");
	if (IS_ERR(vhcd->kthread)) {
		ret = PTR_ERR(vhcd->kthread);
		eprintk("unable to start the thread: %d", ret);
		goto err;
	}

	return 0;
err:
	return ret;
}

/*
 * Helper to stop the worker
 * FIXME: there is a race condition with send_sig and kthread_stop
 */
static inline void
vusb_worker_stop(struct vusb_vhcd *vhcd)
{
	dprintk(D_PM, "Stop the worker\n");

	send_sig(SIGINT, vhcd->kthread, 0); /* To left the function read */
	kthread_stop(vhcd->kthread);
}

/****************************************************************************/
/* Ring Processing                                                          */

static int
vusb_allocate_grefs(struct vusb_device *vdev, struct vusb_urbp *urbp,
		unsigned long *mfns, u32 nr_mfns)
{
	u32 i, ref;
	grant_ref_t gref_head;

	dprintk(D_RING2, "Allocate gref for %d mfns\n", (int)nr_mfns);

	if (xc_gnttab_alloc_grant_references(nr_mfns, &gref_head) < 0) {
		/* TOOD RJP xc_gnttab_request_free_callback(
			&info->callback,
			blkif_restart_queue_callback,
			info,
			BLKIF_MAX_SEGMENTS_PER_REQUEST);*/
		return 1;
	}

	urbp->nr_segments = 0;

	for (i = 0; i < nr_mfns; i++)
	{
		unsigned long mfn = mfns[i];

		ref = xc_gnttab_claim_grant_reference(&gref_head);
		BUG_ON(ref == -ENOSPC);

		urbp->request.gref[urbp->nr_segments] = ref;

		xc_gnttab_grant_foreign_access_ref(ref,
				vdev->xendev->otherend_id, mfn,
				usb_urb_dir_out(urbp->urb)); /* OUT is write, so RO */

		urbp->nr_segments++;
 	}

	xc_gnttab_free_grant_references(gref_head);

	return 0;
}

/****************************************************************************/
/* VUSB Devices                                                             */

static irqreturn_t
vusb_interrupt(int irq, void *dev_id)
{
	/* TODO RJP */
	return IRQ_HANDLED;
}

static void
vusb_device_clear(struct vusb_device *vdev)
{
	if (((vdev->connecting) || (vdev->present)) &&
	    (vdev->xendev != NULL)) 
		dev_set_drvdata(&vdev->xendev->dev, NULL);

	vdev->xendev = NULL;
	vdev->device_id = 0;
	vdev->connecting = 0;
	vdev->present = 0;
}

static void
vusb_usbif_free(struct vusb_device *vdev, int suspend)
{
	/* TODO RJP shut everything down and undo stuff from vusb_talk_to_usbback */
	/* Not sure what all of this will be yet */

	/* Free resources associated with old device channel. */
	if (vdev->ring_ref != GRANT_INVALID_REF) {
		/* This frees the page too */
		xc_gnttab_end_foreign_access(vdev->ring_ref, 0,
					     (unsigned long)vdev->ring.sring);
		vdev->ring_ref = GRANT_INVALID_REF;
		vdev->ring.sring = NULL;
	}

	if (vdev->irq)
		xc_unbind_from_irqhandler(vdev->irq, vdev);
	vdev->evtchn = vdev->irq = 0;
}

static int
vusb_setup_usbfront(struct vusb_device *vdev)
{
	struct xenbus_device *dev = vdev->xendev;
	struct usbif_sring *sring;
	int err;

	vdev->ring_ref = GRANT_INVALID_REF;

	sring = (struct usbif_sring *)__get_free_page(GFP_NOIO | __GFP_HIGH);
	if (!sring) {
		xc_xenbus_dev_fatal(dev, -ENOMEM, "allocating shared ring");
		return -ENOMEM;
	}
	SHARED_RING_INIT(sring);
	FRONT_RING_INIT(&vdev->ring, sring, PAGE_SIZE);

	err = xc_xenbus_grant_ring(dev, virt_to_mfn(vdev->ring.sring));
	if (err < 0) {
		free_page((unsigned long)sring);
		vdev->ring.sring = NULL;
		goto fail;
	}
	vdev->ring_ref = err;

	err = xc_xenbus_alloc_evtchn(dev, &vdev->evtchn);
	if (err)
		goto fail;

	err = xc_bind_evtchn_to_irqhandler(vdev->evtchn, vusb_interrupt,
					USBFRONT_IRQF, "usbif", vdev);
	if (err <= 0) {
		xc_xenbus_dev_fatal(dev, err,
				 "bind_evtchn_to_irqhandler failed");
		goto fail;
	}
	vdev->irq = err;

	return 0;
fail:
	vusb_usbif_free(vdev, 0);
	return err;
}

/* Common code used when first setting up, and when resuming. */
static int
vusb_talk_to_usbback(struct vusb_device *vdev)
{
	struct xenbus_device *dev = vdev->xendev;
	const char *message = NULL;
	struct xenbus_transaction xbt;
	int err;

	/* Create shared ring, alloc event channel. */
	err = vusb_setup_usbfront(vdev);
	if (err)
		goto out;

again:
	err = xc_xenbus_transaction_start(&xbt);
	if (err) {
		xc_xenbus_dev_fatal(dev, err, "starting transaction");
		goto destroy_blkring;
	}

	err = xc_xenbus_printf(xbt, dev->nodename,
	 		    "ring-ref", "%u", vdev->ring_ref);
	if (err) {
		message = "writing ring-ref";
		goto abort_transaction;
	}

	err = xc_xenbus_printf(xbt, dev->nodename,
			    "event-channel", "%u", vdev->evtchn);
	if (err) {
		message = "writing event-channel";
		goto abort_transaction;
	}

	err = xc_xenbus_printf(xbt, dev->nodename, "version", "%d",
			    VUSB_INTERFACE_VERSION);
	if (err) {
		message = "writing protocol";
		goto abort_transaction;
	}

	err = xc_xenbus_transaction_end(xbt, 0);
	if (err) {
		if (err == -EAGAIN)
			goto again;
		xc_xenbus_dev_fatal(dev, err, "completing transaction");
		goto destroy_blkring;
	}

	/* Started out in the initialising state, go to initialised */
	xc_xenbus_switch_state(dev, XenbusStateInitialised);

	return 0;

 abort_transaction:
	xc_xenbus_transaction_end(xbt, 1);
	if (message)
		xc_xenbus_dev_fatal(dev, err, "%s", message);
 destroy_blkring:
	vusb_usbif_free(vdev, 0);
 out:
	return err;
}

static int
vusb_create_device(struct vusb_vhcd *vhcd, struct xenbus_device *dev, u16 id)
{
	u16 i;
	int rc = 0;
	unsigned long flags;
	struct vusb_device *vdev;

	/* Find a port/device we can use. */
	spin_lock_irqsave(&vhcd->lock, flags);
	for (i = 0; i < VUSB_PORTS; i++) {
		if (vhcd->vdev_ports[i].connecting == 0)
			continue;
		if (vhcd->vdev_ports[i].present == 0)
			break;
		if (vhcd->vdev_ports[i].device_id == id) {
			wprintk("Device id 0x%04x already exists on port %d\n",
			       id, vhcd->vdev_ports[i].port);
			rc = -EEXIST;
			goto out;
		}
	}

	if (i >= VUSB_PORTS) {
		printk(KERN_WARNING "Attempt to add a device but no free ports on the root hub.\n");
		rc = -ENOMEM;
		goto out;
	}
	vdev = &vhcd->vdev_ports[i];
	vdev->device_id = id;
	vdev->connecting = 1;
	vdev->parent = vhcd;
	vdev->xendev = dev;
	INIT_LIST_HEAD(&vdev->urbp_list);

	/* Strap our VUSB device onto the Xen device context */
	dev_set_drvdata(&dev->dev, vdev);
	
	/* Setup the rings, event channel and xenstore. Internal failures cleanup
	 * the usbif bits. Wipe the new VUSB dev and bail out.
	 */
	rc = vusb_talk_to_usbback(vdev);
	if (rc) {
		printk(KERN_ERR "Failed to initialize the device - id: %d\n", id);
		vusb_device_clear(vdev);
	}

out:
	spin_unlock_irqrestore(&vhcd->lock, flags);
	return rc;
}

static int
vusb_start_device(struct vusb_device *vdev)
{
	struct vusb_vhcd *vhcd = vdev->parent;
	enum usb_device_speed speed;
	unsigned long flags;
	int rc = 0;

	spin_lock_irqsave(&vhcd->lock, flags);

	/* bits from windows fe for resets and get speed, start sending packets */
	/* final bits from vusb_add_device to make device known*/
	/* TODO RJP get link speed - need to send a packet to the backend to do this */
	vdev->present = 1;
	vdev->connecting = 0;
	vdev->speed = speed;
	vdev->port_status |= usb_speed_to_port_stat(speed)
					 | USB_PORT_STAT_CONNECTION
					 | USB_PORT_STAT_C_CONNECTION << 16;

	dprintk(D_PORT1, "new status: 0x%08x speed: 0x%04x\n",
			vdev->port_status, speed);
	vusb_set_link_state(vdev);
/*out:*/
	spin_unlock_irqrestore(&vhcd->lock, flags);
	usb_hcd_poll_rh_status(vhcd_to_hcd(vhcd));
	return rc;
}

static void
vusb_destroy_device(struct vusb_device *vdev)
{
	struct vusb_vhcd *vhcd = vdev->parent;
	unsigned long flags;

	/* TODO RJP need more than this? */
	spin_lock_irqsave(&vhcd->lock, flags);

	dprintk(D_PORT1, "Remove device from port %u\n", vdev->port);

	vusb_usbif_free(vdev, 0);
	vusb_device_clear(vdev);

	vusb_set_link_state(vdev);

	/* Update hub status */
	vhcd->poll = 1;

	spin_unlock_irqrestore(&vhcd->lock, flags);
}

/**
 * Inititialize a packet header to send
 * @packet: packet to initialize (already allocated)
 * @devid: device id used to discuss with the host
 * @command: what do we want?
 * @hlen: size of header
 * @dlen: size of data
 */
static void
vusb_initialize_packet(struct vusb_vhcd *vhcd, void *packet, u16 devid,
		u8 command, u32 hlen, u32 dlen)
{
	dprintk(D_URB2, "allocate packet len=%u\n",  hlen + dlen);
	/* STUB */
}

/*
 * Send a request to the host
 * A packet is describe as multiple vectors
 */
static int
vusb_send_packet(struct vusb_vhcd *vhcd, const struct iovec *iovec, size_t niov)
{
	/* TODO this was a r = vusb_write(v, iovec[i].iov_base, iovec[i].iov_len);*/

	return 0;
}

/* Process packet received from vusb daemon */
static int
vusb_process_packet(struct vusb_vhcd *vhcd, const void *packet)
{
	int res = 0;

	switch (vhcd->state) {
	case VUSB_WAIT_BIND_RESPONSE:
		iprintk("Wait bind response send it\n");

		/* TODO this is all v4v stuffs, fix res = vusb_send_bind_commit(v);*/
		if (res != 0) {
			eprintk("Failed to send bind commit command\n");
			return res;
		}
		break;

	case VUSB_RUNNING:
		/* STUB handle events calling one of:
		vusb_handle_announce_device(v, packet);
		vusb_handle_device_gone(v, packet);
		vusb_handle_urb_response(v, packet);
		vusb_handle_urb_status(v, packet);*/
		break;

	default:
		wprintk("Invalid state %u in process_packet\n",	vhcd->state);
		return -1;
	}

	return 0;
}

/*
 * Send a reset command
 * TODO: Add return value and check return
 */
static void
vusb_send_reset_device_cmd(struct vusb_vhcd *vhcd, struct vusb_device *vdev)
{
	vusb_create_packet(iovec, 1);
	void *packet;

	if (!vdev->present) {
		wprintk("Ignore reset for not present device port %u\n", vdev->port);
		vdev->reset = 0;
		vusb_set_link_state(vdev);
		return;
	}

	dprintk(D_URB2, "Send reset command, port = %u\n", vdev->port);

	vusb_initialize_packet(vhcd, &packet, vdev->device_id,
			0 /* STUB reset internal command */,
			0 /* STUB reset length */,
			0);

	vusb_set_packet_header(iovec, &packet, 0 /* STUB reset length */);
	vusb_send_packet(vhcd, iovec, 1);

	/* Signal reset completion */
	vdev->port_status |= (USB_PORT_STAT_C_RESET << 16);

	vusb_set_link_state(vdev);
	vhcd->poll = 1;
}

/****************************************************************************/
/* VUSB Xen Devices & Driver                                                */

static int
vusb_usbfront_probe(struct xenbus_device *dev, const struct xenbus_device_id *id)
{
	struct vusb_vhcd *vhcd = hcd_to_vhcd(platform_get_drvdata(vusb_platform_device));
	int vid, err;


	/* Make device ids out of the virtual-device value from xenstore */
	err = xc_xenbus_scanf(XBT_NIL, dev->nodename, "virtual-device", "%i", &vid);
	if (err != 1) {
		printk(KERN_ERR "Failed to read virtual-device value\n");
		return err;
	}

	printk(KERN_INFO "Creating new VUSB device - virtual-device: %i devicetype: %s\n",
		vid, id->devicetype);

	return vusb_create_device(vhcd, dev, (u16)vid);
}

/**
 * Callback received when the backend's state changes.
 */
static void
vusb_usbback_changed(struct xenbus_device *dev, enum xenbus_state backend_state)
{
	struct vusb_device *vdev = dev_get_drvdata(&dev->dev);

	dev_dbg(&dev->dev, "%s\n", xc_xenbus_strstate(backend_state));
	dev_dbg(&dev->dev, "Mine: %s\n", xc_xenbus_strstate(dev->state));

	switch (backend_state) {
	case XenbusStateUnknown:
		/* if the backend vanishes from xenstore, close frontend */
		if (!xc_xenbus_exists(XBT_NIL, dev->otherend, "")) {
			/* Gone is gone, don't care about our state since we do not reconnect
			 * devices. Just destroy the device.
			 */
			printk(KERN_INFO "backend vanished, closing frontend\n");
			xc_xenbus_switch_state(dev, XenbusStateClosed);
			vusb_destroy_device(vdev);
		}
		break;
	case XenbusStateInitialising:
	case XenbusStateInitialised:
	case XenbusStateReconfiguring:
	case XenbusStateReconfigured:
		break;
	case XenbusStateConnected:
		if (vusb_start_device(vdev)) {
			printk(KERN_ERR "failed to start frontend, aborting!\n");
			xc_xenbus_switch_state(dev, XenbusStateClosed);
			vusb_destroy_device(vdev);
		}
		break;

	case XenbusStateInitWait:
		if (dev->state != XenbusStateInitialising && dev->state != XenbusStateClosed)
			break;
		/* Frontend drives the backend from InitWait to Connected */
		xc_xenbus_switch_state(dev, XenbusStateConnected);
		break;

	case XenbusStateClosing:
	case XenbusStateClosed:
		/* Remove the device and transition ourselves to closed, there is no
		 * reconnect. Do it for the closed state just in case the backend never
		 * transitioned through closing.
		 */
		xc_xenbus_switch_state(dev, XenbusStateClosed);
		vusb_destroy_device(vdev);
		break;
	}
}

static int
vusb_xenusb_remove(struct xenbus_device *dev)
{
	/* TODO RJP our ctx: struct netfront_info *info = dev_get_drvdata(&dev->dev);*/

	dev_dbg(&dev->dev, "%s\n", dev->nodename);

	/* TODO RJP this causes vusb device removal - not sure what to do here yet
	xennet_disconnect_backend(info);
	del_timer_sync(&info->rx_refill_timer);
	xennet_sysfs_delif(info->netdev);
	unregister_netdev(info->netdev);
	free_netdev(info->netdev);*/

	return 0;
}

static int
vusb_usbfront_suspend(struct xenbus_device *dev)
{
	/* TODO RJP our ctx: struct netfront_info *info = dev_get_drvdata(&dev->dev);*/

	mutex_lock(&vusb_xen_pm_mutex);
	printk(KERN_INFO "xen_netif: pm freeze event received, detaching netfront\n");
	/* TODO RJP not sure what to do here yet
        info->suspending = 1;

	spin_lock_bh(&info->rx_lock);
	spin_lock_irq(&info->tx_lock);
	netif_carrier_off(info->netdev);
	spin_unlock_irq(&info->tx_lock);
	spin_unlock_bh(&info->rx_lock);

	xennet_disconnect_backend(info);
	xennet_uninit(info->netdev);
	*/

	mutex_unlock(&vusb_xen_pm_mutex);
	return 0;
}

static int
vusb_usbfront_resume(struct xenbus_device *dev)
{
	int err = 0;
	/* TODO RJP our ctx: struct netfront_info *info = dev_get_drvdata(&dev->dev);*/

	mutex_lock(&vusb_xen_pm_mutex);
	printk(KERN_INFO "xen_usbif: pm restore event received, unregister net device\n");
	/* TODO RJP not sure what to do here yet
        info->suspending = 0;
	err = xennet_init_rings(info);
	if (!err)
		xc_xenbus_switch_state(dev, XenbusStateClosed);
	mutex_unlock(&xennet_pm_mutex);*/
	return err;
}

static struct xenbus_device_id vusb_usbfront_ids[] = {
	{ "vusb" },
	{ "" }
};

static struct xenbus_driver vusb_usbfront_driver = {
	.name = "xc-vusb",
	.owner = THIS_MODULE,
	.ids = vusb_usbfront_ids,
	.probe = vusb_usbfront_probe,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0))
	.remove = vusb_xenusb_remove,
#else
	.remove = __devexit_p(vusb_xenusb_remove),
#endif

	.suspend = vusb_usbfront_suspend,
	.resume = vusb_usbfront_resume,

	.otherend_changed = vusb_usbback_changed,
};

static int
vusb_xen_init(void)
{

	int rc = 0;

	mutex_lock(&vusb_xen_pm_mutex);
	if (!xen_hvm_domain()) {
		rc = -ENODEV;
		goto out;
	}

	rc = xenbus_register_frontend(&vusb_usbfront_driver);
	if (rc)
		goto out;

	printk(KERN_INFO "xen_usbif initialized\n");
out:
	mutex_unlock(&vusb_xen_pm_mutex);
	return rc;
}

static void
vusb_xen_unregister(void)
{
	mutex_lock(&vusb_xen_pm_mutex);
	xc_xenbus_unregister_driver(&vusb_usbfront_driver);
	mutex_unlock(&vusb_xen_pm_mutex);
}

/****************************************************************************/
/* VUSB Platform Device & Driver                                            */

/* Platform probe */
static int
vusb_platform_probe(struct platform_device *pdev)
{
	struct usb_hcd *hcd;
	int retval;
	struct vusb_vhcd *vhcd;

	if (usb_disabled())
		return -ENODEV;

	dprintk(D_MISC, ">vusb_hcd_probe\n");
	dev_info(&pdev->dev, "%s, driver " VUSB_DRIVER_VERSION "\n", VUSB_DRIVER_DESC);

	hcd = usb_create_hcd(&vusb_hcd_driver, &pdev->dev, dev_name(&pdev->dev));
	if (!hcd)
		return -ENOMEM;

	/* Indicate the USB stack that both Super and Full Speed are supported */
	hcd->has_tt = 1;

	vhcd = hcd_to_vhcd(hcd);

	spin_lock_init(&vhcd->lock);
	INIT_LIST_HEAD(&vhcd->urbp_list);

	retval = vusb_worker_start(vhcd);
	if (retval != 0)
		goto err_worker;

	retval = usb_add_hcd(hcd, 0, 0);
	if (retval != 0)
		goto err_add;

	dprintk(D_MISC, "<vusb_hcd_probe %d\n", retval);

	return 0;

err_add:
	vusb_worker_stop(vhcd);
err_worker:
	usb_put_hcd(hcd);

	dprintk(D_MISC, "<vusb_hcd_probe %d\n", retval);

	return retval;
}

/* Platform remove */
static int
vusb_platform_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd;
	struct vusb_vhcd *vhcd;

	hcd = platform_get_drvdata(pdev);

	/*
	 * A warning will result: "IRQ 0 already free".
	 * It seems the linux kernel doesn't set hcd->irq to -1 when IRQ
	 * is not enabled for a USB driver. So we put an hack for this
	 * before calling usb_remove_hcd().
	 */
	hcd->irq = -1;

	usb_remove_hcd(hcd);

	vhcd = hcd_to_vhcd(hcd);

	/* Stop the main thread and release its memory */
	vusb_worker_stop(vhcd);

	usb_put_hcd(hcd);

	return 0;
}

#ifdef CONFIG_PM
/*
 * Platform freeze
 * Called during hibernation process
 */
static int
vusb_platform_freeze(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct usb_hcd *hcd;
	struct vusb_vhcd *vhcd;
	int rc = 0;
	unsigned long flags;

	iprintk("HCD freeze\n");

	hcd = platform_get_drvdata(pdev);
	vhcd = hcd_to_vhcd(hcd);
	spin_lock_irqsave(&vhcd->lock, flags);

	dprintk(D_PM, "root hub state %s (%u)\n", vusb_rhstate_to_string(vhcd),
		vhcd->rh_state);

	if (vhcd->rh_state == VUSB_RH_RUNNING) {
		wprintk("Root hub isn't suspended!\n");
		rc = -EBUSY;
	} else
		clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
	spin_unlock_irqrestore(&vhcd->lock, flags);

	if (rc == 0)
		vusb_worker_stop(vhcd);

	return rc;
}

/* Platform restore */
static int
vusb_platform_restore(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct usb_hcd *hcd;
	unsigned long flags;
	struct vusb_vhcd *vhcd;
	int rc = 0;

	iprintk("HCD restore\n");

	hcd = platform_get_drvdata(pdev);
	vhcd = hcd_to_vhcd(hcd);

	spin_lock_irqsave(&vhcd->lock, flags);
	set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
	spin_unlock_irqrestore(&vhcd->lock, flags);

	rc = vusb_worker_start(vhcd);
	if (rc != 0)
		usb_hcd_poll_rh_status(hcd);

	return rc;
}
#endif /* CONFIG_PM */

#ifdef CONFIG_PM
static const struct dev_pm_ops vusb_platform_pm = {
	.freeze = vusb_platform_freeze,
	.restore = vusb_platform_restore,
	.thaw = vusb_platform_restore,
};
#endif /* CONFIG_PM */

static struct platform_driver vusb_platform_driver = {
	.probe = vusb_platform_probe,
	.remove = vusb_platform_remove,
	.driver = {
		.name = VUSB_PLATFORM_DRIVER_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &vusb_platform_pm,
#endif /* CONFIG_PM */
	},
};

/****************************************************************************/
/* Module Init & Cleanup                                                    */

static void
vusb_cleanup(void)
{
	iprintk("clean up\n");
	platform_device_unregister(vusb_platform_device);
	platform_driver_unregister(&vusb_platform_driver);
}

static int __init
vusb_init(void)
{
	int r = 0;

	iprintk("OpenXT USB host controller\n");

	if (usb_disabled()) {
		wprintk("USB is disabled\n");
		return -ENODEV;
	}

	vusb_platform_device = platform_device_alloc(VUSB_PLATFORM_DRIVER_NAME, -1);
	if (!vusb_platform_device) {
		eprintk("Unable to allocate platform device\n");
		return -ENOMEM;
	}

	r = platform_driver_register(&vusb_platform_driver);
	if (r < 0) {
		eprintk("Unable to register the platform\n");
		goto err_driver_register;
	}

	r = platform_device_add(vusb_platform_device);
	if (r < 0) {
		eprintk("Unable to add the platform\n");
		goto err_add_hcd;
	}

	return 0;

err_add_hcd:
	platform_driver_unregister(&vusb_platform_driver);
err_driver_register:
	platform_device_put(vusb_platform_device);

	return r;
}

module_init(vusb_init);
module_exit(vusb_cleanup);

MODULE_DESCRIPTION("Xen virtual USB frontend");
MODULE_LICENSE ("GPL");
