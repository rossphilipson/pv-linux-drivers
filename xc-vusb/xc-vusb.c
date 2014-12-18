/******************************************************************************
 * vusb.c
 *
 * OpenXT vUSB frontend driver
 *
 * Copyright (c) 2014 Ross Philipson <ross.philipson@gmail.com>
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
 *** - Add branch prediction
 *** - Devices are not kept accross suspend/hibernate (vusb daemon issue)
 *** - Reorganize the code
 ***/

#include <linux/mm.h>
#include <linux/version.h>
#include <linux/module.h>
#include <asm/uaccess.h>
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

#if ( LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,32) )
#define flush_work_sync(a) flush_scheduled_work()
#endif

#define VUSB_INTERFACE_VERSION 	3
#define VUSB_MAX_PACKET_SIZE 	1024*256

#define VUSB_PLATFORM_DRIVER_NAME	"vusb-platform"
#define VUSB_HCD_DRIVER_NAME		"vusb-hcd"
#define VUSB_DRIVER_DESC		"OpenXT Virtual USB Host Controller"
#define VUSB_DRIVER_VERSION		"1.0.0"

#define POWER_BUDGET	5000 /* mA */

#define GRANT_INVALID_REF 0
#define USB_RING_SIZE __CONST_RING_SIZE(usbif, PAGE_SIZE)
#define SHADOW_ENTRIES USB_RING_SIZE

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

/* TODO these will go away, use USBIF_RSP_USB_* codes.
 *Status codes */
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
};

/* Internal request structure */
struct vusb_internal {
	u8 			type;
	u8 			endpoint;
	void 			*page;
	usbif_request_len_t 	length;
	u16 			offset;
	unsigned		is_reset:1;
};

struct usbif_indirect_pages {
	unsigned long		frames[USBIF_MAX_SEGMENTS_PER_IREQUEST];
};
typedef struct usbif_indirect_pages usbif_indirect_pages_t;

struct vusb_shadow {
	usbif_request_t		req;
	unsigned long		frames[USBIF_MAX_SEGMENTS_PER_REQUEST];
	struct vusb_urbp	*urbp;
	void			*iso_packet_descriptor;
	void			*indirect_page_memory;
	void			*indirect_pages;
    	unsigned		in_use:1;
    	unsigned		is_reset:1;
};

/* Virtual USB device on of the RH ports */
struct vusb_device {
	u16				device_id;
	u32				port_status;
	u16				address;
	u16				port;
	enum usb_device_speed		speed;
	unsigned			present:1;
	unsigned			connecting:1;
	unsigned			processing:1;
	unsigned			closing:1;
	unsigned			reset:2;

	/* The Xenbus device associated with this vusb device */
	struct xenbus_device    	*xendev;

	/* Pointer back to the virtual HCD core device */
	struct vusb_vhcd		*vhcd;

	/* Lock for device resources */
	spinlock_t			lock;

	/* Xen rings and event channel */
	int				ring_ref;
	struct usbif_front_ring 	ring;
	unsigned int 			evtchn;
	unsigned int 			irq;
	struct gnttab_free_callback 	callback;

	/* Shadow buffers */
	struct vusb_shadow		*shadows;
	u16				*shadow_free_list;
	u16				shadow_free;

	/* This VUSB device's list of pending URBs */
	struct list_head        	pending_list;
	struct list_head        	release_list;

	struct work_struct 		work;

	wait_queue_head_t		wait_queue;
};

/* Virtual USB HCD/RH pieces */
enum vusb_rh_state {
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

	u16				urb_handle;

	/*
	 * Update hub can't be done in critical section.
	 * Is the driver need to update the hub?
	 */
	unsigned			poll:1;
};

static struct platform_device *vusb_platform_device = NULL;
static DEFINE_MUTEX(vusb_xen_pm_mutex);

static bool
vusb_start_processing(struct vusb_device *vdev);
static void
vusb_stop_processing(struct vusb_device *vdev);
static void
vusb_restart_processing(struct work_struct *work);
static void
vusb_process_requests(struct vusb_device *vdev, struct vusb_urbp *urbp);
static int
vusb_send_packet(struct vusb_device *vdev, const struct iovec *iovec, size_t niov);
static void
vusb_initialize_packet(struct vusb_device *vdev, void *packet,
		u8 command, u32 hlen, u32 dlen);
static void
vusb_send_reset_device_cmd(struct vusb_device *vdev);

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
	case VUSB_RH_SUSPENDED:
		return "SUSPENDED";
	case VUSB_RH_RUNNING:
		return "RUNNING";
	default:
		return "UNKNOWN";
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

static inline u16
vusb_speed_to_port_stat(enum usb_device_speed speed)
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
					vusb_speed_to_port_stat(vdev->speed);
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

	/* TODO RJP vusb_worker_notify(vhcd);*/
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

	/* TODO this will probably go away and we will use the req.id */
	if (vhcd->urb_handle >= 0xfff0)
		/* reset to 0 we never have lots URB in the list */
		vhcd->urb_handle = 0;

	return vhcd->urb_handle;
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

static void
vusb_urbp_queue_release(struct vusb_device *vdev, struct vusb_urbp *urbp)
{
	/* Remove from the active urbp list and place it on the release list.
	 * This is called from the urb processing routines holding the vdev lock.
	 */
	list_del(&urbp->urbp_list);

	list_add_tail(&urbp->urbp_list, &vdev->release_list);
	
	schedule_work(&vdev->work);
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

static int
vusb_init_hcd(struct vusb_vhcd *vhcd)
{
	int i;

	dprintk(D_PM, "Init the HCD\n");

	/* TODO RJP revisit, may not want to clear the devices on resume path
	 * also may need suspend/resume for S3 */
	/* Initialize ports */
	for (i = 0; i < VUSB_PORTS; i++) {
		vhcd->vdev_ports[i].port = i + 1;
		vhcd->vdev_ports[i].vhcd = vhcd;
		vhcd->vdev_ports[i].connecting = 0;
		vhcd->vdev_ports[i].present = 0;
		vhcd->vdev_ports[i].processing = 0;
		vhcd->vdev_ports[i].closing = 0;
		spin_lock_init(&vhcd->vdev_ports[i].lock);
		INIT_WORK(&vhcd->vdev_ports[i].work, vusb_restart_processing);
	}

	vhcd->state = VUSB_INACTIVE;

	return 0;
}

/* HCD start */
static int
vusb_hcd_start(struct usb_hcd *hcd)
{
	struct vusb_vhcd *vhcd = hcd_to_vhcd(hcd);

	iprintk("XEN HCD start\n");

	dprintk(D_MISC, ">vusb_start\n");

	vhcd->rh_state = VUSB_RH_RUNNING;
	vhcd->state = VUSB_RUNNING;

	hcd->power_budget = POWER_BUDGET;
	hcd->state = HC_STATE_RUNNING;
	hcd->uses_new_polling = 1;

	dprintk(D_MISC, "<vusb_start 0\n");

	return 0;
}

/* HCD stop */
static void
vusb_hcd_stop(struct usb_hcd *hcd)
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
	struct vusb_device *vdev;
	int ret = -ENOMEM;

	dprintk(D_MISC, ">vusb_urb_enqueue\n");

	vhcd = hcd_to_vhcd(hcd);

	if (!urb->transfer_buffer && urb->transfer_buffer_length)
		return -EINVAL;

	urbp = kmalloc(sizeof(*urbp), mem_flags);
	if (!urbp)
		return -ENOMEM;

	urbp->state = VUSB_URBP_NEW;
	/* Port numbered from 1 */
	urbp->port = urb->dev->portnum;
	urbp->urb = urb;

	spin_lock_irqsave(&vhcd->lock, flags);
	/* Allocate a handle as a unique ID */
	urbp->handle = vusb_get_urb_handle(vhcd);

	vdev = vusb_device_by_port(vhcd, urbp->port);
	if (vhcd->state == VUSB_INACTIVE || !vdev->present) {
		eprintk("Enqueue start processing called while device(s) in invalid states\n");
		kfree(urbp);
		spin_unlock_irqrestore(&vhcd->lock, flags);
		return -ESHUTDOWN;
	}

	if (vdev->closing) {
		kfree(urbp);
		spin_unlock_irqrestore(&vhcd->lock, flags);
		return -ESHUTDOWN;
	}

	ret = usb_hcd_link_urb_to_ep(hcd, urb);
	if (ret) {
		kfree(urbp);
		spin_unlock_irqrestore(&vdev->lock, flags);
		return ret;
	}

	/* Set it in the processing state so it is not nuked out from under us */
	vdev->processing = 1;
	spin_unlock_irqrestore(&vhcd->lock, flags);

	vusb_process_requests(vdev, urbp);

	/* Finished processing */
	vusb_stop_processing(vdev);

	return 0;
}

static int
vusb_hcd_urb_dequeue(struct usb_hcd *hcd, struct urb *urb, int status)
{
	struct vusb_vhcd *vhcd;
	unsigned long flags;
	int ret;
	struct vusb_device *vdev;
	struct vusb_urbp *urbp;

	dprintk(D_MISC, "*vusb_urb_dequeue\n");
	vhcd = hcd_to_vhcd(hcd);

	spin_lock_irqsave(&vhcd->lock, flags);

	ret = usb_hcd_check_unlink_urb(hcd, urb, status);
	if (ret) {
		spin_unlock_irqrestore(&vhcd->lock, flags);
		return ret;
	}

	urb->status = status;

	/* If it can't be processed, the urbp and urb will be released
	 * in the device teardown code which is where this device is going
	 * (or gone).
	 */
	vdev = vusb_device_by_port(vhcd, urb->dev->portnum);
	if (vhcd->state == VUSB_INACTIVE || !vdev->present) {
		eprintk("Dequeue start processing called while device(s) in invalid states\n");
		spin_unlock_irqrestore(&vhcd->lock, flags);
		return -ESHUTDOWN;
	}

	if (vdev->closing) {
		spin_unlock_irqrestore(&vhcd->lock, flags);
		return -ESHUTDOWN;
	}

	/* Set it in the processing state so it is not nuked out from under us */
	vdev->processing = 1;
	spin_unlock_irqrestore(&vhcd->lock, flags);

	spin_lock_irqsave(&vdev->lock, flags);

	/* Retrieve URBp */
	list_for_each_entry(urbp, &vdev->pending_list, urbp_list) {
		if (urbp->urb == urb)
			break;
	}

	if (!urbp) { 
		wprintk("Try do dequeue an unhandle URB\n");
		spin_unlock_irqrestore(&vdev->lock, flags);
		return -ENODEV;
	}

	/* TODO I think I am wrong about using the abort. We may need another
	 * inernal command to cancel at the URB level. The backend would call
	 * something like usb_kill_urb or usb_unlink_urb. Leaving the earlier
	 * note in case...
	 *
	 * TODO this may actually be an abort using the internal abort request
	 * USBIF_T_ABORT_PIPE. At this level it may simply be setting another
	 * state and processing. Down below it may require sometime like
	 * ProcessAbortPipe or AbortEndpointWorker. Not really sure why there
	 * are two of them, needs investigation.
	 *
	 * Oh and note, this is exactly what ctxusb used to do with CTXUSB_URBP_CANCEL
	 * which is an internal request. So abort is probably the right thing here.
	 */
	urbp->state = VUSB_URBP_CANCEL;
	spin_unlock_irqrestore(&vdev->lock, flags);

	vusb_process_requests(vdev, NULL);

	/* Finished processing */
	vusb_stop_processing(vdev);

	return 0;
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
	int ret = 0;

	dprintk(D_PM, "Bus resume\n");

	spin_lock_irq(&vhcd->lock);
	if (!HCD_HW_ACCESSIBLE(hcd)) {
		ret = -ESHUTDOWN;
	} else {
		vhcd->rh_state = VUSB_RH_RUNNING;
		vhcd->state = VUSB_RUNNING;
		hcd->state = HC_STATE_RUNNING;
	}
	spin_unlock_irq(&vhcd->lock);

	return ret;
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
/* Ring Processing                                                          */

static struct vusb_shadow*
vusb_get_shadow(struct vusb_device *vdev)
{
	if (!vdev->shadow_free) {
		printk(KERN_ERR "Requesting shadow when shadow_free == 0!\n");
		return NULL;
	}

	vdev->shadow_free--;

	if (vdev->shadows[vdev->shadow_free_list[vdev->shadow_free]].in_use) {
		printk(KERN_ERR "Requesting shadow at %d which is in use!\n",
			vdev->shadow_free);
		return NULL;
	}

	vdev->shadows[vdev->shadow_free_list[vdev->shadow_free]].in_use = 1;
	vdev->shadows[vdev->shadow_free_list[vdev->shadow_free]].req.nr_segments = 0;
	vdev->shadows[vdev->shadow_free_list[vdev->shadow_free]].req.nr_packets = 0;
	vdev->shadows[vdev->shadow_free_list[vdev->shadow_free]].req.flags = 0;
	vdev->shadows[vdev->shadow_free_list[vdev->shadow_free]].req.length = 0;

	return &vdev->shadows[vdev->shadow_free_list[vdev->shadow_free]];
}

static void
vusb_put_shadow(struct vusb_device *vdev, struct vusb_shadow *shadow)
{
	usbif_indirect_request_t *ireq;
	int i, j;

	if (!shadow->in_use) {
		printk(KERN_ERR "Returning shadow %p that is not in use to list!\n",
			shadow);
		return;
	}

	/* Free any resources in use */
	if (shadow->iso_packet_descriptor) {
		kfree(shadow->iso_packet_descriptor);
		shadow->iso_packet_descriptor = NULL;
	}

	if (shadow->indirect_page_memory) {
		ireq = (usbif_indirect_request_t*)shadow->indirect_page_memory;

		for (i = 0; i < shadow->req.nr_segments; i++) {
			for (j = 0; j < ireq[j].nr_segments; j++) {
				xc_gnttab_end_foreign_access(ireq[i].gref[j], 0, 0UL);
			}
		}
		kfree(shadow->indirect_page_memory);
		shadow->indirect_page_memory = NULL;
		kfree(shadow->indirect_pages);
		shadow->indirect_pages = NULL;
	}

	for (i = 0; i < shadow->req.nr_segments; i++) {
		xc_gnttab_end_foreign_access(shadow->req.gref[i], 0, 0UL);
	}

	memset(&shadow->frames[0], 0,
		(sizeof(unsigned long)*USBIF_MAX_SEGMENTS_PER_REQUEST));
	shadow->req.nr_segments = 0;
	shadow->urbp = NULL;
	shadow->in_use = 0;

	if (vdev->shadow_free >= SHADOW_ENTRIES) {
		printk(KERN_ERR "Shadow free value too big: %d!\n",
			vdev->shadow_free);
		return;
	}

	vdev->shadow_free_list[vdev->shadow_free] = (u16)shadow->req.id;
	vdev->shadow_free++;
}

static void
vusb_flush_requests(struct vusb_device *vdev)
{
	int notify;

	RING_PUSH_REQUESTS_AND_CHECK_NOTIFY(&vdev->ring, notify);

	if (notify)
		xc_notify_remote_via_irq(vdev->irq);
}

static void
vusb_put_ring(struct vusb_device *vdev, struct vusb_shadow *shadow)
{
	usbif_request_t *req;

	/* If we have a shadow allocation, we know there is space on the ring */	
	req = RING_GET_REQUEST(&vdev->ring, vdev->ring.req_prod_pvt);
	memcpy(req, &shadow->req, sizeof(usbif_request_t));

	vdev->ring.req_prod_pvt++;

	vusb_flush_requests(vdev);
}

static void
vusb_restart_processing_callback(void *arg)
{
	struct vusb_device *vdev = (struct vusb_device*)arg;
	schedule_work(&vdev->work);
}

static int
vusb_allocate_grefs(struct vusb_device *vdev, struct vusb_shadow *shadow,
		unsigned long *mfns, u32 nr_mfns, bool restart)
{
	u32 i, ref;
	grant_ref_t gref_head;
	int ret;

	dprintk(D_RING2, "Allocate gref for %d mfns\n", (int)nr_mfns);

	/* NOTE: we are following the blkback model where we are not
	 * freeing the pages on gnttab_end_foreign_access so we don't
	 * have to track them. That memory belongs to the USB core
	 * in most cases or is the internal request page.
	 */

	ret = xc_gnttab_alloc_grant_references(nr_mfns, &gref_head);
	if (ret < 0) {
		if (!restart)
			return ret;
		xc_gnttab_request_free_callback(&vdev->callback,
			vusb_restart_processing_callback,
			vdev, nr_mfns);
		return -EBUSY;
	}

	for (i = 0; i < nr_mfns; i++) {
		unsigned long mfn = mfns[i];

		ref = xc_gnttab_claim_grant_reference(&gref_head);
		BUG_ON(ref == -ENOSPC);

		shadow->req.gref[shadow->req.nr_segments] = ref;

		xc_gnttab_grant_foreign_access_ref(ref,
				vdev->xendev->otherend_id, mfn,
				usb_urb_dir_out(shadow->urbp->urb)); /* OUT is write, so RO */

		shadow->frames[i] = mfn_to_pfn(mfn);
		shadow->req.nr_segments++;
 	}

	xc_gnttab_free_grant_references(gref_head);

	return 0;
}

static int
vusb_put_internal_request(struct vusb_device *vdev, struct vusb_internal *vint)
{
	struct vusb_shadow *shadow;
	unsigned long mfn;
	int ret = 0;

	/* Internal request can use the last entry */
	if (vdev->shadow_free == 0)
		return -ENOMEM;

	/* All internal requests fit on a page */
	mfn = pfn_to_mfn(virt_to_phys(vint->page) << PAGE_SHIFT);

	shadow = vusb_get_shadow(vdev);
	BUG_ON(!shadow);

	if (vint->is_reset) {
		shadow->req.endpoint = 0;
		shadow->req.type = 0;
		shadow->req.length = 0;
		shadow->req.offset = 0;
		shadow->req.flags = USBIF_F_RESET;
		shadow->is_reset = 1;
	}
	else {
		shadow->req.endpoint = vint->endpoint;
		shadow->req.type = vint->type;
		shadow->req.length = vint->length;
		BUG_ON(vint->offset >= 0x00010000);
		shadow->req.offset = vint->offset;
		shadow->req.flags = USBIF_F_SHORTOK;
		shadow->is_reset = 0;
	}

	shadow->req.nr_segments = 0;
	shadow->req.setup = 0L;

	/* Get some of them grefs */
	ret = vusb_allocate_grefs(vdev, shadow, &mfn, 1, false);
	if (ret) {
		/* Can't handle failures */
		vusb_put_shadow(vdev, shadow);
		return ret;
	}
	
	vusb_put_ring(vdev, shadow);

	return ret;
}

/****************************************************************************/
/* URB Processing                                                           */

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

/* Dump the URBp list */
static inline void
vusb_urbp_list_dump(const struct vusb_device *vdev, const char *fn)
{
	const struct vusb_urbp *urbp;

	dprintk(D_URB2, "===== Current URB List in %s =====\n", fn);
	list_for_each_entry(urbp, &vdev->pending_list, urbp_list) {
		dprintk(D_URB1, "URB handle 0x%x port %u device %u\n",
			urbp->handle, urbp->port, vdev->device_id);
	}
	dprintk(D_URB2, "===== End URB List in %s ====\n", fn);
}

/* Retrieve a URB by handle */
static struct vusb_urbp*
vusb_urb_by_handle(struct vusb_device *vdev, u16 handle)
{
	struct vusb_urbp *urbp;

	list_for_each_entry(urbp, &vdev->pending_list, urbp_list) {
		if (urbp->handle == handle)
			return urbp;
	}

	dprintk(D_URB1, "Unable to retrieve URB handle 0x%x port %u\n",
		handle, vdev->port);
	vusb_urbp_list_dump(vdev, __FUNCTION__);

	return NULL;
}

/* Common part to finish an URB request */
static void
vusb_urb_common_finish(struct vusb_device *vdev, struct vusb_urbp *urbp, bool in,
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

	vusb_urbp_queue_release(vdev, urbp);
}

/*
 * Finish an isochronous URB
 */
static void
vusb_urb_isochronous_finish(struct vusb_device *vdev, struct vusb_urbp *urbp,
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

	vusb_urbp_queue_release(vdev, urbp);
	return;

iso_err:
	urb->status = -EIO;
	for (i = 0; i < urb->number_of_packets; i++) {
		urb->iso_frame_desc[i].actual_length = 0;
		urb->iso_frame_desc[i].status = urb->status;
	}
	urb->actual_length = 0;

	vusb_urbp_queue_release(vdev, urbp);
}

/* Finish a control URB */
static void
vusb_urb_control_finish(struct vusb_device *vdev, struct vusb_urbp *urbp, u32 len, const u8 *data)
{
	const struct usb_ctrlrequest *ctrl;
	bool in;

	ctrl = (struct usb_ctrlrequest *)urbp->urb->setup_packet;

	in = (ctrl->bRequestType & USB_DIR_IN) != 0;

	vusb_urb_common_finish(vdev, urbp, in, len, data);
}

/* Finish a bulk URB */
static void
vusb_urb_bulk_finish(struct vusb_device *vdev, struct vusb_urbp *urbp, u32 len, const u8 *data)
{
	vusb_urb_common_finish(vdev, urbp, usb_urb_dir_in(urbp->urb), len, data);
}

/* Finish an interrupt URB */
static void
vusb_urb_interrupt_finish(struct vusb_device *vdev, struct vusb_urbp *urbp, u32 len, const u8 *data)
{
	vusb_urb_common_finish(vdev, urbp, usb_urb_dir_in(urbp->urb), len, data);
}

/* Convert status to errno */
static int
vusb_urb_status_to_errno(u32 status)
{
	int32_t st = status;

	/* TODO this will be changed to map USBIF_RSP_USB_* to errnos
	 * sort of like the Windows version MapUsbifToUsbdStatus
	 */
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

	/* TODO this will be changed to use USBIF_RSP_USB_* if
	 * it evens sticks around.
	 */
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
vusb_urb_finish(struct vusb_device *vdev, u16 handle,
		u32 status, u32 len, const u8 *data)
{
	struct vusb_urbp *urbp;
	struct urb *urb;

	urbp = vusb_urb_by_handle(vdev, handle);

	if (!urbp) {
		dprintk(D_WARN, "Bad handle (0x%x) for Device ID (%u)\n",
			handle, vdev->device_id);
		return;
	}

	urb = urbp->urb;
	urb->status = vusb_urb_status_to_errno(status);

	switch (usb_pipetype(urb->pipe)) {
	case PIPE_ISOCHRONOUS:
		vusb_urb_isochronous_finish(vdev, urbp, len, data);
		break;

	case PIPE_CONTROL:
		vusb_urb_control_finish(vdev, urbp, len, data);
		break;

	case PIPE_INTERRUPT:
		vusb_urb_interrupt_finish(vdev, urbp, len, data);
		break;

	case PIPE_BULK:
		vusb_urb_bulk_finish(vdev, urbp, len, data);
		break;

	default:
		wprintk("Unknow pipe type %u\n",
			usb_pipetype(urb->pipe));
	}
}

/* Handle command URB response
 *
 * TOP-LEVEL
 */
static void
vusb_handle_urb_response(struct vusb_device *vdev, const void *packet)
{
	u16 handle = 0 /* STUB logical handle */;
	u32 status = 0;
	u32 len = 0;

	/* STUB sanity check and get response values */

	/* TODO RJP another one we dont need */
	vusb_urb_finish(vdev, handle, status, len, NULL /* STUB the response data */);
}

/* Handle command URB status
 *
 * TOP-LEVEL
 */
static void
vusb_handle_urb_status(struct vusb_device *vdev, const void *packet)
{
	u16 handle = 0 /* STUB logical handle */;
	u32 status = 0;

	/* STUB sanity check and get status values */

	/* RJP lock held, probably don't even need this */
	vusb_urb_finish(vdev, handle, status, 0, NULL);
}

/*
 * Initialize an URB packet
 * @packet: packet to initialize (already allocated)
 * @command: what do we want?
 * @hlen: size of header
 * @has_data: if true, length will be the one of the transfer buffer
 */
static void
vusb_initialize_urb_packet(void *packet,
		const struct vusb_urbp *urbp, struct vusb_device *vdev,
		u8 command, u32 hlen, bool has_data)
{
	if (has_data) /* Outbound request */
		vusb_initialize_packet(vdev, packet,
				command, hlen, urbp->urb->transfer_buffer_length);
	else
		vusb_initialize_packet(vdev, packet,
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
vusb_send_urb_packet(struct vusb_device *vdev, struct vusb_urbp *urbp, 
		void *packet, u32 hlen, bool has_data)
{
	vusb_create_packet(iovec, 2);
	int r;

	vusb_set_packet_header(iovec, packet, hlen);
	if (has_data)
		vusb_set_packet_data(iovec, urbp->urb->transfer_buffer,
				urbp->urb->transfer_buffer_length);

	r = vusb_send_packet(vdev, iovec, (has_data) ? 2 : 1);

	if (r < 0) {
		/* An error occured drop the URB and notify the USB stack */
		urbp->state = VUSB_URBP_DROP;
		urbp->urb->status = r;
	} else
		urbp->state = VUSB_URBP_SENT;

	return r;
}


/* Not defined by hcd.h */
#define InterfaceOutRequest 						\
	((USB_DIR_OUT|USB_TYPE_STANDARD|USB_RECIP_INTERFACE) << 8)

/* Send an urb control to the host */
static void
vusb_send_control_urb(struct vusb_device *vdev, struct vusb_urbp *urbp)
{
	struct urb *urb = urbp->urb;
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

	/* TODO RJP this probably doesn't make sense now. This was split out for
	 * the ctxusb protocol. Not sure what it will look like for us.
	 */
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
		vusb_initialize_urb_packet(&packet, urbp, vdev,
				0 /* STUB set configuration internal command */,
				hlen, false);

		/* STUB finish packet setup */
		break;

	case InterfaceOutRequest | USB_REQ_SET_INTERFACE:
		hlen = 0 /* STUB select interface length */;
		vusb_initialize_urb_packet(&packet, urbp, vdev,
				0 /* STUB select interface  internal command */,
				hlen, false);

		/* STUB finish packet setup */
		break;

	default:
		hlen = 0 /* STUB control length */;
		vusb_initialize_urb_packet(&packet, urbp, vdev,
				0 /* STUB control internal command */,
				hlen, !in);

		/* STUB finish packet setup */
	}

	vusb_send_urb_packet(vdev, urbp, &packet, hlen, has_data /* STUB may or may not have data in packet */);
}

/* Send an URB interrup command */
static void
vusb_send_interrupt_urb(struct vusb_device *vdev, struct vusb_urbp *urbp)
{
	struct urb *urb = urbp->urb;
	void *packet;

	dprintk(D_URB2, "Send Interrupt URB Device: %u Endpoint: %u in: %u\n",
		usb_pipedevice(urb->pipe),
		usb_pipeendpoint(urb->pipe),
		usb_urb_dir_in(urb));

	vusb_initialize_urb_packet(&packet, urbp, vdev,
			0 /* STUB interrupt internal command */,
			0 /* STUB interrupt length */,
			usb_urb_dir_out(urb));

	/* STUB finish packet setup */

	vusb_send_urb_packet(vdev, urbp, &packet,
			0 /* STUB interrupt length */,
			usb_urb_dir_out(urb));
}

/* Send an URB bulk command */
static void
vusb_send_bulk_urb(struct vusb_device *vdev, struct vusb_urbp *urbp)
{
	struct urb *urb = urbp->urb;
	void *packet;

	dprintk(D_URB2, "Send Bulk URB Device: %u Endpoint: %u in: %u\n",
		usb_pipedevice(urb->pipe),
		usb_pipeendpoint(urb->pipe),
		usb_urb_dir_in(urb));

	vusb_initialize_urb_packet(&packet, urbp, vdev,
			0 /* STUB bulk internal command */,
			0 /* STUB bulk length */,
			usb_urb_dir_out(urb));

	/* STUB finish packet setup */

	vusb_send_urb_packet(vdev, urbp, &packet,
			0 /* STUB bulk length */,
			usb_urb_dir_out(urb));
}

/* Send an isochronous urb command */
static void
vusb_send_isochronous_urb(struct vusb_device *vdev, struct vusb_urbp *urbp)
{
	struct urb *urb = urbp->urb;
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

	/* Use the common urb initialization packet but fix ByteCount */
	vusb_initialize_urb_packet(&packet, urbp, vdev,
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

	r = vusb_send_packet(vdev, iovec, (usb_urb_dir_out(urb)) ? 3 : 2);

	if (r < 0) {
		/* An error occured drop the URB and notify the USB stack */
		urbp->state = VUSB_URBP_DROP;
		urbp->urb->status = r;
	} else
		urbp->state = VUSB_URBP_SENT;
}

/* Send a cancel URB command */
static void
vusb_send_cancel_urb(struct vusb_device *vdev, struct vusb_urbp *urbp)
{
	vusb_create_packet(iovec, 1);
	void *packet;

	/* TODO RJP this may be a special something that needs internal processing */
	/* TODO and yes, this is an abort in vusb. This will get replaced with internal
	 * requests. */
	vusb_initialize_packet(vdev, &packet,
			0 /* STUB cancel internal command */,
			0 /* STUB cancel length */,
			0);

	/* STUB finish packet setup */

	dprintk(D_URB1, "send packet URB_CANCEL device %u port %u handle 0x%04x\n",
		vdev->device_id, vdev->port, urbp->handle);

	vusb_set_packet_header(iovec, &packet, 0 /* STUB cancel length */);
	vusb_send_packet(vdev, iovec, 1);
}

/* Send an URB */
static void
vusb_send_urb(struct vusb_device *vdev, struct vusb_urbp *urbp)
{
	struct urb *urb = urbp->urb;
	unsigned int type;

	type = usb_pipetype(urb->pipe);

	dprintk(D_URB2, "urb handle: 0x%x status: %s pipe: %s(%u)\n",
		urbp->handle, vusb_state_to_string(urbp),
		vusb_pipe_to_string(urb), type);

	if (urbp->state == VUSB_URBP_NEW) {
		switch (type) {
		case PIPE_ISOCHRONOUS:
			vusb_send_isochronous_urb(vdev, urbp);
			break;

		case PIPE_INTERRUPT:
			vusb_send_interrupt_urb(vdev, urbp);
			break;

		case PIPE_CONTROL:
			vusb_send_control_urb(vdev, urbp);
			break;

		case PIPE_BULK:
			vusb_send_bulk_urb(vdev, urbp);
			break;

		default:
			wprintk("Unknown urb type %x\n", type);
		}
	} else if (urbp->state == VUSB_URBP_CANCEL) {
		vusb_send_cancel_urb(vdev, urbp);
	}

	if (urbp->state == VUSB_URBP_DONE ||
	    urbp->state == VUSB_URBP_DROP ||
	    urbp->state == VUSB_URBP_CANCEL) {
		/* Remove URB */
		dprintk(D_URB1, "URB immediate %s\n",
			vusb_state_to_string(urbp));
		vusb_urbp_queue_release(vdev, urbp);
	}
}

/****************************************************************************/
/* VUSB Devices                                                             */

static bool
vusb_start_processing(struct vusb_device *vdev)
{
	struct vusb_vhcd *vhcd = vdev->vhcd;
	unsigned long flags;

	spin_lock_irqsave(&vhcd->lock, flags);

	if (vhcd->state == VUSB_INACTIVE || !vdev->present) {
		eprintk("Start processing called while device(s) in invalid statesi\n");
		spin_unlock_irqrestore(&vhcd->lock, flags);
		return false;
	}

	if (vdev->closing) {
		/* Normal, shutdown of this device pending */
		spin_unlock_irqrestore(&vhcd->lock, flags);
		return false;
	}

	vdev->processing = 1;
	spin_unlock_irqrestore(&vhcd->lock, flags);

	return true;
}

static void
vusb_stop_processing(struct vusb_device *vdev)
{
	struct vusb_vhcd *vhcd = vdev->vhcd;
	unsigned long flags;

	spin_lock_irqsave(&vhcd->lock, flags);
	vdev->processing = 0;
	spin_unlock_irqrestore(&vhcd->lock, flags);
}

static void
vusb_wait_stop_processing(struct vusb_device *vdev)
{
	struct vusb_vhcd *vhcd = vdev->vhcd;
	unsigned long flags;

again:
	spin_lock_irqsave(&vhcd->lock, flags);

	vdev->closing = 0; /* Goning away now... */

	if (vdev->processing) {
		spin_unlock_irqrestore(&vhcd->lock, flags);
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(HZ);
		goto again;
	}

	spin_unlock_irqrestore(&vhcd->lock, flags);
}

static void
vusb_check_reset_devices(struct vusb_vhcd *vhcd)
{
	unsigned long flags;
	int i;

	/* TODO RJP this needs some work - may not be ok - need to queue? */
	spin_lock_irqsave(&vhcd->lock, flags);

	/* Check if we need to reset a device */
	for (i = 0; i < VUSB_PORTS; i++) {
		if (vhcd->vdev_ports[i].reset == 1) {
			vhcd->vdev_ports[i].reset = 2;
			vusb_send_reset_device_cmd(&vhcd->vdev_ports[i]);
		}
	}

	spin_unlock_irqrestore(&vhcd->lock, flags);
}

/* Notify USB stack that the URB is finished and release it. This
 * has to be done outside the all locks.
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
	kfree(urbp);
	usb_hcd_unlink_urb_from_ep(vhcd_to_hcd(vhcd), urb);
	usb_hcd_giveback_urb(vhcd_to_hcd(vhcd), urb, urb->status);
}

static void
vusb_process_requests(struct vusb_device *vdev, struct vusb_urbp *urbp)
{
	struct vusb_vhcd *vhcd = vdev->vhcd;
	struct vusb_urbp *pos;
	struct vusb_urbp *next;
	struct list_head tmp;
	unsigned long flags;

	/* TODO RJP check elsewhere like in work/bh */
	vusb_check_reset_devices(vhcd);

	spin_lock_irqsave(&vdev->lock, flags);

	/* New URB, queue it at the back */
	if (urbp)
		list_add_tail(&urbp->urbp_list, &vdev->pending_list);

	/* Drive request processing */
	list_for_each_entry_safe(pos, next, &vdev->pending_list, urbp_list) {
		/* TODO RJP fix to schedule work if we cannot drain the queue */
		vusb_send_urb(vdev, pos);
	}

	/* Copy off any urbps on the release list that need releasing */
        list_splice_init(&vdev->release_list, &tmp);

	spin_unlock_irqrestore(&vdev->lock, flags);

	/* Clean them up outside the lock */
	list_for_each_entry_safe(pos, next, &tmp, urbp_list) {
		vusb_urbp_release(vhcd, pos);
	}

	/* TODO RJP check elsewhere like in work/bh */
	if (vhcd->poll) { /* Update Hub status */
		vhcd->poll = 0;
		usb_hcd_poll_rh_status(vhcd_to_hcd(vhcd));
	}
}

static void
vusb_restart_processing(struct work_struct *work)
{
	struct vusb_device *vdev = container_of(work, struct vusb_device, work);

	if (!vusb_start_processing(vdev))
		return;

	/* TODO RJP Start response processing again */

	/* Start request processing again */
	vusb_process_requests(vdev, NULL);

	vusb_stop_processing(vdev);
}

static irqreturn_t
vusb_interrupt(int irq, void *dev_id)
{
	struct vusb_device *vdev = (struct vusb_device*)dev_id;
	unsigned long flags;

	if (!vusb_start_processing(vdev))
		return IRQ_HANDLED;

	/* TODO RJP */
	/* TODO process the results from internal requests in here */
	spin_lock_irqsave(&vdev->lock, flags);
	spin_unlock_irqrestore(&vdev->lock, flags);

	vusb_stop_processing(vdev);

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
	vdev->processing = 0;
}

static void
vusb_usbif_free(struct vusb_device *vdev, int suspend)
{
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

	if (vdev->shadows) {
		kfree(vdev->shadows);
		vdev->shadows = NULL;
	}

	if (vdev->shadow_free_list) {
		kfree(vdev->shadow_free_list);
		vdev->shadow_free_list = NULL;
	}
}

static int
vusb_setup_usbfront(struct vusb_device *vdev)
{
	struct xenbus_device *dev = vdev->xendev;
	struct usbif_sring *sring;
	int err, i;

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

	/* Allocate the shadow buffers */
	vdev->shadows = kzalloc(sizeof(struct vusb_shadow)*SHADOW_ENTRIES,
				GFP_KERNEL);
	if (!vdev->shadows) {
		xc_xenbus_dev_fatal(dev, err,
				 "allocate shadows failed");
		err = -ENOMEM;
		goto fail;
	}
	
	vdev->shadow_free_list = kzalloc(sizeof(u16)*SHADOW_ENTRIES,
				GFP_KERNEL);
	if (!vdev->shadow_free_list) {
		xc_xenbus_dev_fatal(dev, err,
				 "allocate shadow free list failed");
		err = -ENOMEM;
		goto fail;
	}

	for (i = 0; i < SHADOW_ENTRIES; i++) {
		vdev->shadows[i].req.id = i;
		vdev->shadows[i].in_use = 1;
		vusb_put_shadow(vdev, &vdev->shadows[i]);
	}

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
	int ret = 0;
	unsigned long flags;
	struct vusb_device *vdev;

	/* Find a port/device we can use. */
	spin_lock_irqsave(&vhcd->lock, flags);
	for (i = 0; i < VUSB_PORTS; i++) {
		if ((vhcd->vdev_ports[i].connecting)||(
		    (vhcd->vdev_ports[i].closing)))
			continue;
		if (!vhcd->vdev_ports[i].present)
			break;
		if (vhcd->vdev_ports[i].device_id == id) {
			wprintk("Device id 0x%04x already exists on port %d\n",
			       id, vhcd->vdev_ports[i].port);
			ret = -EEXIST;
			goto out;
		}
	}

	if (i >= VUSB_PORTS) {
		wprintk("Attempt to add a device but no free ports on the root hub.\n");
		ret = -ENOMEM;
		goto out;
	}

	vdev = &vhcd->vdev_ports[i];
	vdev->device_id = id;
	vdev->connecting = 1;
	vdev->vhcd = vhcd;
	vdev->xendev = dev;
	INIT_LIST_HEAD(&vdev->pending_list);
	INIT_LIST_HEAD(&vdev->release_list);
	init_waitqueue_head(&vdev->wait_queue);

	/* Strap our VUSB device onto the Xen device context */
	dev_set_drvdata(&dev->dev, vdev);
	
	/* Setup the rings, event channel and xenstore. Internal failures cleanup
	 * the usbif bits. Wipe the new VUSB dev and bail out.
	 */
	ret = vusb_talk_to_usbback(vdev);
	if (ret) {
		printk(KERN_ERR "Failed to initialize the device - id: %d\n", id);
		vusb_device_clear(vdev);
	}

out:
	spin_unlock_irqrestore(&vhcd->lock, flags);
	return ret;
}

static int
vusb_start_device(struct vusb_device *vdev)
{
	struct vusb_vhcd *vhcd = vdev->vhcd;
	struct vusb_internal vint_req;
	unsigned long flags;
	int ret = 0;

	vint_req.type = USBIF_T_GET_SPEED;
	vint_req.endpoint = 0 | USB_DIR_IN;
	vint_req.page = (void *)__get_free_page(GFP_NOIO | __GFP_HIGH);
	vint_req.length = sizeof(u32);
	vint_req.offset = 0;
	vint_req.is_reset = 0;

	if (!vint_req.page)
		return -ENOMEM;
	
	/* TODO need a reset here? The WFE gets that info from the registry */

	spin_lock_irqsave(&vhcd->lock, flags);


	vdev->present = 1;
	vdev->connecting = 0;
	vdev->speed = (unsigned int)-1;

	ret = vusb_put_internal_request(vdev, &vint_req);
	if (ret) {
		spin_unlock_irqrestore(&vhcd->lock, flags);
		free_page((unsigned long)vint_req.page);
		eprintk("Failed to get device speed - ret: %d\n", ret);
		return ret;
	}
	spin_unlock_irqrestore(&vhcd->lock, flags);

	/* Wait for a response with no lock */
	wait_event_interruptible(vdev->wait_queue, vdev->speed != (unsigned int)-1);

	spin_lock_irqsave(&vhcd->lock, flags);

	/* TODO this will happen in the irq vdev->speed = speed, sanity check the speed */
	vdev->port_status |= vusb_speed_to_port_stat(vdev->speed)
					 | USB_PORT_STAT_CONNECTION
					 | USB_PORT_STAT_C_CONNECTION << 16;

	/* final bits from ctxusb_add_device to make device known*/
	vusb_set_link_state(vdev);
	dprintk(D_PORT1, "new status: 0x%08x speed: 0x%04x\n",
			vdev->port_status, vdev->speed);

	spin_unlock_irqrestore(&vhcd->lock, flags);

	free_page((unsigned long)vint_req.page);
	usb_hcd_poll_rh_status(vhcd_to_hcd(vhcd));


	return 0;
}

static void
vusb_destroy_device(struct vusb_device *vdev)
{
	struct vusb_vhcd *vhcd = vdev->vhcd;
	struct list_head tmp[2];
	struct vusb_urbp *pos;
	struct vusb_urbp *next;
	unsigned long flags;

	dprintk(D_PORT1, "Remove device from port %u\n", vdev->port);

	INIT_LIST_HEAD(&tmp[0]);
	INIT_LIST_HEAD(&tmp[1]);

	/* Disconnect gref free callback so it schedules no more work */
	xc_gnttab_cancel_free_callback(&vdev->callback);

	/* Shutdown all work. Must be done with no locks held. */
	flush_work_sync(&vdev->work);

	/* Wait for all processing to stop now */
	vusb_wait_stop_processing(vdev);

	/* Final device operations */
	spin_lock_irqsave(&vdev->lock, flags);

	/* Give usbback a chance to consume the ring */
	vusb_flush_requests(vdev);

	/* Copy ready to release urbps to temp list */
        list_splice_init(&vdev->release_list, &tmp[0]);

	/* Copy pending urbps to temp list */
        list_splice_init(&vdev->pending_list, &tmp[1]);

	spin_unlock_irqrestore(&vdev->lock, flags);

	/* Release all the ready to release URBs and pending URBs - this
	 * has to be done outside a lock
	 */
	list_for_each_entry_safe(pos, next, &tmp[0], urbp_list) {
		vusb_urbp_release(vhcd, pos);
	}

	list_for_each_entry_safe(pos, next, &tmp[1], urbp_list) {
		pos->urb->status = -ESHUTDOWN;
		vusb_urbp_release(vhcd, pos);
	}

	spin_lock_irqsave(&vhcd->lock, flags);

	/* Final VHCD operations on device */
	vusb_usbif_free(vdev, 0);
	vusb_device_clear(vdev);

	vusb_set_link_state(vdev);

	if (vhcd->state != VUSB_INACTIVE)
		vhcd->poll = 1;

	spin_unlock_irqrestore(&vhcd->lock, flags);

	/* Update hub status, device gone, port empty */
	if (vhcd->poll) {
		vhcd->poll = 0;
		usb_hcd_poll_rh_status(vhcd_to_hcd(vhcd));
	}
}

/**
 * Inititialize a packet header to send
 * @packet: packet to initialize (already allocated)
 * @devid: device id used to discuss with the host
 * @command: what do we want?
 * @hlen: size of header
 * @dlen: size of data
 * TODO RJP nuke it...
 */
static void
vusb_initialize_packet(struct vusb_device *vdev, void *packet,
			u8 command, u32 hlen, u32 dlen)
{
	dprintk(D_URB2, "allocate packet len=%u\n",  hlen + dlen);
	/* STUB */
}

/*
 * Send a request to the host
 * A packet is describe as multiple vectors
 * TODO RJP nuke it and all those packet macros...
 */
static int
vusb_send_packet(struct vusb_device *vdev, const struct iovec *iovec, size_t niov)
{
	/* TODO this was a r = vusb_write(v, iovec[i].iov_base, iovec[i].iov_len);*/

	return 0;
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
	 * Outbound path fe -> be
	 *
	 * Moved in with the device functions. Something like this will
	 * become a device function driven by enqueue and work. The kthread
	 * will go away.
	 *
	 * Update: this thing is a mess. It is both the inbound and outbound
	 * processor. Just leave it here for reference.
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

/* Process packet received from vusb daemon */
static int
vusb_process_packet(struct vusb_vhcd *vhcd, const void *packet)
{
	int res = 0;

	/* TODO RJP
	 *
	 * Inbound path: be -> fe
	 *
	 * This will also become a device function driven by the irq/bh/work
	 *
	 */
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
vusb_send_reset_device_cmd(struct vusb_device *vdev)
{
	vusb_create_packet(iovec, 1);
	void *packet;

	/* TODO this is an internal request too, this will get replaced */
	if (!vdev->present) {
		wprintk("Ignore reset for not present device port %u\n", vdev->port);
		vdev->reset = 0;
		vusb_set_link_state(vdev);
		return;
	}

	dprintk(D_URB2, "Send reset command, port = %u\n", vdev->port);

	vusb_initialize_packet(vdev, &packet, 
			0 /* STUB reset internal command */,
			0 /* STUB reset length */,
			0);

	vusb_set_packet_header(iovec, &packet, 0 /* STUB reset length */);
	vusb_send_packet(vdev, iovec, 1);

	/* Signal reset completion */
	vdev->port_status |= (USB_PORT_STAT_C_RESET << 16);

	vusb_set_link_state(vdev);
	vdev->vhcd->poll = 1;
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
	 * maybe something like blkif_recover - that is why I kept the frame[] arrays around
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

static void
vusb_platform_cleanup(struct vusb_vhcd *vhcd)
{
	unsigned long flags;
	u16 i = 0;

	dprintk(D_PM, "Clean up the worker\n");

	spin_lock_irqsave(&vhcd->lock, flags);
	vhcd->state = VUSB_INACTIVE;
	spin_unlock_irqrestore(&vhcd->lock, flags);

	/* Unplug all USB devices */
	for (i = 0; i < VUSB_PORTS; i++)
		xc_xenbus_switch_state(vhcd->vdev_ports[i].xendev, XenbusStateClosed);
}

/* Platform probe */
static int
vusb_platform_probe(struct platform_device *pdev)
{
	struct usb_hcd *hcd;
	int ret;
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

	ret = usb_add_hcd(hcd, 0, 0);
	if (ret != 0)
		goto err_add;

	vusb_init_hcd(vhcd);

	dprintk(D_MISC, "<vusb_hcd_probe %d\n", retval);

	return 0;

err_add:
	usb_put_hcd(hcd);

	dprintk(D_MISC, "<vusb_hcd_probe %d\n", retval);

	return ret;
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

	vusb_platform_cleanup(vhcd);

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
	int ret = 0;
	unsigned long flags;

	iprintk("HCD freeze\n");

	hcd = platform_get_drvdata(pdev);
	vhcd = hcd_to_vhcd(hcd);
	spin_lock_irqsave(&vhcd->lock, flags);

	dprintk(D_PM, "root hub state %s (%u)\n", vusb_rhstate_to_string(vhcd),
		vhcd->rh_state);

	if (vhcd->rh_state == VUSB_RH_RUNNING) {
		wprintk("Root hub isn't suspended!\n");
		ret = -EBUSY;
	} else
		clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
	spin_unlock_irqrestore(&vhcd->lock, flags);

	if (ret == 0)
		vusb_platform_cleanup(vhcd);

	return ret;
}

/* Platform restore */
static int
vusb_platform_restore(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct usb_hcd *hcd;
	unsigned long flags;
	struct vusb_vhcd *vhcd;

	iprintk("HCD restore\n");

	hcd = platform_get_drvdata(pdev);
	vhcd = hcd_to_vhcd(hcd);

	spin_lock_irqsave(&vhcd->lock, flags);
	set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
	vusb_init_hcd(vhcd);
	spin_unlock_irqrestore(&vhcd->lock, flags);

	return 0;
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
