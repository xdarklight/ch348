// SPDX-License-Identifier: GPL-2.0
/*
 * USB serial driver for USB to Octal UARTs chip ch348.
 *
 * Copyright (C) 2022 Corentin Labbe <clabbe@baylibre.com>
 * With the help of Neil Armstrong <neil.armstrong@linaro.org>
 * and the help of Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 */

#include <linux/completion.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/overflow.h>
#include <linux/serial.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/usb.h>
#include <linux/usb/serial.h>

#define CH348_CMD_TIMEOUT   2000

#define CH348_CTO_D	0x01
#define CH348_CTO_R	0x02

#define CH348_CTI_C	0x10
#define CH348_CTI_DSR	0x20
#define CH348_CTI_R	0x40
#define CH348_CTI_DCD	0x80

#define CH348_LO	0x02
#define CH348_LP	0x04
#define CH348_LF	0x08
#define CH348_LB	0x10

#define CMD_W_R		0xC0
#define CMD_W_BR	0x80

#define CMD_WB_E	0x90
#define CMD_RB_E	0xC0

#define M_NOR		0x00
#define M_HF		0x03

#define R_MOD		0x97
#define R_IO_D		0x98
#define R_IO_O		0x99
#define R_IO_I		0x9b
#define R_TM_O		0x9c
#define R_INIT		0xa1

#define R_C1		0x01
#define R_C2		0x02
#define R_C4		0x04
#define R_C5		0x06

#define R_II_B1		0x06
#define R_II_B2		0x02
#define R_II_B3		0x00

#define CMD_VER		0x96

#define CH348_RX_PORT_CHUNK_LENGTH	32
#define CH348_RX_PORT_MAX_LENGTH	30

struct ch348_rxbuf {
	u8 port;
	u8 length;
	u8 data[CH348_RX_PORT_MAX_LENGTH];
} __packed;

struct ch348_txbuf {
	u8 port;
	__le16 length;
	u8 data[];
} __packed;

#define CH348_TX_HDRSIZE offsetof(struct ch348_txbuf, data)

struct ch348_initbuf {
	u8 cmd;
	u8 reg;
	u8 port;
	__be32 baudrate;
	u8 format;
	u8 paritytype;
	u8 databits;
	u8 rate;
	u8 unknown;
} __packed;

#define CH348_MAXPORT 8

/*
 * The CH348 multiplexes rx & tx into a pair of Bulk USB endpoints for
 * the 8 serial ports, and another pair of Bulk USB endpoints to
 * set port settings and receive port status events.
 *
 * The USB serial cores ties every Bulk endpoints pairs to each ports,
 * but in our case it will set port 0 with the rx/tx endpoints
 * and port 1 with the setup/status endpoints.
 *
 * To still take advantage of the generic code, we (re-)initialize
 * the USB serial port structure with the correct USB endpoint
 * for read and write, and write proper process_read_urb()
 * and prepare_write_buffer() to correctly (de-)multiplex data.
 * Also we use a custom write() implementation to wait until the buffer
 * has been fully transmitted to prevent TX buffer overruns.
 */

/*
 * struct ch348_port - per-port information
 * @uartmode:		UART port current mode
 * @write_completion:	completion event when the TX buffer has been written out
 */
struct ch348_port {
	u8 uartmode;
	struct completion write_completion;
};

/*
 * struct ch348 - main container for all this driver information
 * @udev:		pointer to the CH348 USB device
 * @ports:		List of per-port information
 * @serial:		pointer to the serial structure
 * @write_lock:		protect against concurrent writes so we don't lose data
 * @cmd_ep:		endpoint number for configure operations
 * @status_urb:		URB for status
 * @status_buffer:	buffer used by status_urb
 */
struct ch348 {
	struct usb_device *udev;
	struct ch348_port ports[CH348_MAXPORT];
	struct usb_serial *serial;

	struct mutex write_lock;

	int cmd_ep;

	struct urb *status_urb;
	u8 status_buffer[];
};

struct ch348_magic {
	u8 action;
	u8 reg;
	u8 control;
} __packed;

struct ch348_status_entry {
	u8 portnum:4;
	u8 unused:4;
	u8 reg_iir;
	union {
		u8 lsr_signal;
		u8 modem_signal;
		u8 init_data[10];
	};
} __packed;

static void ch348_process_status_urb(struct urb *urb)
{
	struct ch348_status_entry *status_entry;
	struct ch348 *ch348 = urb->context;
	int ret, status = urb->status;
	struct usb_serial_port *port;
	unsigned int i, status_len;

	switch (status) {
	case 0:
		/* success */
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		/* this urb is terminated, clean up */
		dev_dbg(&urb->dev->dev, "%s - urb shutting down with status: %d\n",
			__func__, status);
		return;
	default:
		dev_err(&urb->dev->dev, "%s - nonzero urb status received: %d\n",
			__func__, status);
		goto exit;
	}

	if (urb->actual_length < 3) {
		dev_warn(&ch348->udev->dev,
			 "Received too short status buffer with %u bytes\n",
			 urb->actual_length);
		goto exit;
	}

	for (i = 0; i < urb->actual_length;) {
		status_entry = urb->transfer_buffer + i;

		if (status_entry->portnum >= CH348_MAXPORT) {
			dev_warn(&ch348->udev->dev,
				 "Invalid port %d in status entry\n",
				 status_entry->portnum);
			break;
		}

		port = ch348->serial->port[status_entry->portnum];
		status_len = 3;

		if (!status_entry->reg_iir) {
			dev_dbg(&port->dev, "Ignoring status with zero reg_iir\n");
		} else if (status_entry->reg_iir == R_INIT) {
			status_len = 12;
		} else if ((status_entry->reg_iir & 0x0f) == R_II_B1) {
			if (status_entry->lsr_signal & CH348_LO)
				port->icount.overrun++;
			if (status_entry->lsr_signal & CH348_LP)
				port->icount.parity++;
			if (status_entry->lsr_signal & CH348_LF)
				port->icount.frame++;
			if (status_entry->lsr_signal & CH348_LF)
				port->icount.brk++;
		} else if ((status_entry->reg_iir & 0x0f) == R_II_B2) {
			complete_all(&ch348->ports[status_entry->portnum].write_completion);
		} else {
			dev_warn(&port->dev,
				 "Unsupported status with reg_iir 0x%02x\n",
				 status_entry->reg_iir);
		}

		usb_serial_debug_data(&port->dev, __func__, status_len,
				      urb->transfer_buffer + i);

		i += status_len;
	}

exit:
	ret = usb_submit_urb(urb, GFP_ATOMIC);
	if (ret)
		dev_err(&urb->dev->dev, "%s - usb_submit_urb failed; %d\n",
			__func__, ret);
}

/*
 * Some values came from vendor tree, and we have no meaning for them, this
 * function simply use them.
 */
static int ch348_do_magic(struct ch348 *ch348, int portnum, u8 action, u8 reg, u8 control)
{
	struct ch348_magic *buffer;
	int ret, len;

	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	if (portnum < 4)
		reg += 0x10 * portnum;
	else
		reg += 0x10 * (portnum - 4) + 0x08;

	buffer->action = action;
	buffer->reg = reg;
	buffer->control = control;

	ret = usb_bulk_msg(ch348->udev, ch348->cmd_ep, buffer, 3, &len,
			   CH348_CMD_TIMEOUT);
	if (ret)
		dev_err(&ch348->udev->dev, "Failed to write magic err=%d\n", ret);

	kfree(buffer);

	return ret;
}

static int ch348_configure(struct ch348 *ch348, int portnum)
{
	int ret;

	ret = ch348_do_magic(ch348, portnum, CMD_W_R, R_C2, 0x87);
	if (ret)
		return ret;

	return ch348_do_magic(ch348, portnum, CMD_W_R, R_C4, 0x08);
}

static void ch348_process_read_urb(struct urb *urb)
{
	struct usb_serial_port *port = urb->context;
	struct ch348 *ch348 = usb_get_serial_data(port->serial);
	unsigned int portnum, usblen, i;
	struct ch348_rxbuf *rxb;

	if (urb->actual_length < 2) {
		dev_dbg(&ch348->udev->dev, "Empty rx buffer\n");
		return;
	}

	for (i = 0; i < urb->actual_length; i += CH348_RX_PORT_CHUNK_LENGTH) {
		rxb = urb->transfer_buffer + i;
		portnum = rxb->port;
		if (portnum >= CH348_MAXPORT) {
			dev_dbg(&ch348->udev->dev, "Invalid port %d\n", portnum);
			break;
		}

		port = ch348->serial->port[portnum];

		usblen = rxb->length;
		if (usblen > CH348_RX_PORT_MAX_LENGTH) {
			dev_dbg(&port->dev, "Invalid length %d for port %d\n",
				usblen, portnum);
			break;
		}

		tty_insert_flip_string(&port->port, rxb->data, usblen);
		tty_flip_buffer_push(&port->port);
		port->icount.rx += usblen;
		usb_serial_debug_data(&port->dev, __func__, usblen, rxb->data);
	}
}

static int ch348_prepare_write_buffer(struct usb_serial_port *port, void *dest, size_t size)
{
	struct ch348_txbuf *rxt = dest;
	int count;

	count = kfifo_out_locked(&port->write_fifo, rxt->data,
				 size - CH348_TX_HDRSIZE, &port->lock);

	rxt->port = port->port_number;
	rxt->length = cpu_to_le16(count);

	return count + CH348_TX_HDRSIZE;
}

static int ch348_write(struct tty_struct *tty, struct usb_serial_port *port,
		       const unsigned char *buf, int count)
{
	struct ch348 *ch348 = usb_get_serial_data(port->serial);
	struct ch348_port *ch348_port = &ch348->ports[port->port_number];
	int ret, max_tx_size;

	if (tty_get_baud_rate(tty) < 9600 && count >= 128)
		/*
		 * Writing larger buffers can take longer than the hardware
		 * allows before discarding the write buffer. Limit the
		 * transfer size in such cases.
		 * These values have been found by empirical testing.
		 */
		max_tx_size = 128;
	else
		/*
		* Only ingest as many bytes as we can transfer with one URB at
		* a time. Once an URB has been written we need to wait for the
		* R_II_B2 status event before we are allowed to send more data.
		* If we ingest more data then usb_serial_generic_write() will
		* internally try to process as much data as possible with any
		* number of URBs without giving us the chance to wait in
		* between transfers.
		*/
		max_tx_size = port->bulk_out_size - CH348_TX_HDRSIZE;

	reinit_completion(&ch348_port->write_completion);

	mutex_lock(&ch348->write_lock);

	/*
	 * For any (remaining) bytes that we did not transfer TTY core will
	 * call us again, with the buffer and count adjusted to the remaining
	 * data.
	 */
	ret = usb_serial_generic_write(tty, port, buf, min(count, max_tx_size));

	mutex_unlock(&ch348->write_lock);

	if (ret <= 0)
		return ret;

	if (!wait_for_completion_interruptible_timeout(&ch348_port->write_completion,
						       msecs_to_jiffies(CH348_CMD_TIMEOUT))) {
		dev_err_console(port, "Failed to wait for TX buffer flush\n");
		return -ETIMEDOUT;
	}

	return ret;
}

static int ch348_set_uartmode(struct ch348 *ch348, int portnum, u8 mode)
{
	int ret;

	if (ch348->ports[portnum].uartmode == M_NOR && mode == M_HF) {
		ret = ch348_do_magic(ch348, portnum, CMD_W_BR, R_C4, 0x51);
		if (ret)
			return ret;
		ch348->ports[portnum].uartmode = M_HF;
	}

	if (ch348->ports[portnum].uartmode == M_HF && mode == M_NOR) {
		ret = ch348_do_magic(ch348, portnum, CMD_W_BR, R_C4, 0x50);
		if (ret)
			return ret;
		ch348->ports[portnum].uartmode = M_NOR;
	}
	return 0;
}

static void ch348_set_termios(struct tty_struct *tty, struct usb_serial_port *port,
			      const struct ktermios *termios_old)
{
	struct ch348 *ch348 = usb_get_serial_data(port->serial);
	struct ktermios *termios = &tty->termios;
	int ret, portnum = port->port_number;
	struct ch348_initbuf *buffer;
	speed_t	baudrate;
	u8 format;

	if (termios_old && !tty_termios_hw_change(&tty->termios, termios_old))
		return;

	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (!buffer) {
		if (termios_old)
			tty->termios = *termios_old;
		return;
	}

	/*
	 * The datasheet states that only baud rates in range of 1200..6000000
	 * are supported. Tests however show that even baud rates as low as 50
	 * and as high as 12000000 are working in practice.
	 */
	baudrate = clamp(tty_get_baud_rate(tty), 50, 12000000);

	format = termios->c_cflag & CSTOPB ? 2 : 1;

	buffer->paritytype = 0;
	if (termios->c_cflag & PARENB) {
		if (termios->c_cflag & PARODD)
			buffer->paritytype += 1;
		else
			buffer->paritytype += 2;
		if  (termios->c_cflag & CMSPAR)
			buffer->paritytype += 2;
	}

	switch (C_CSIZE(tty)) {
	case CS5:
		buffer->databits = 5;
		break;
	case CS6:
		buffer->databits = 6;
		break;
	case CS7:
		buffer->databits = 7;
		break;
	case CS8:
	default:
		buffer->databits = 8;
		break;
	}
	buffer->cmd = CMD_WB_E | (portnum & 0x0F);
	buffer->reg = R_INIT;
	buffer->port = portnum;
	buffer->baudrate = cpu_to_be32(baudrate);

	if (format == 2)
		buffer->format = 0x02;
	else if (format == 1)
		buffer->format = 0x00;

	buffer->rate = max_t(speed_t, 5, (10000 * 15 / baudrate) + 1);

	ret = usb_bulk_msg(ch348->udev, ch348->cmd_ep, buffer,
			   sizeof(*buffer), NULL, CH348_CMD_TIMEOUT);
	if (ret < 0) {
		dev_err(&ch348->udev->dev, "Failed to change line settings: err=%d\n",
			ret);
		goto out;
	}

	ret = ch348_do_magic(ch348, portnum, CMD_W_R, R_C1, 0x0F);
	if (ret < 0)
		goto out;

	if (C_CRTSCTS(tty))
		ret = ch348_set_uartmode(ch348, portnum, M_HF);
	else
		ret = ch348_set_uartmode(ch348, portnum, M_NOR);

out:
	kfree(buffer);
}

static int ch348_open(struct tty_struct *tty, struct usb_serial_port *port)
{
	struct ch348 *ch348 = usb_get_serial_data(port->serial);
	int ret;

	if (tty)
		ch348_set_termios(tty, port, NULL);

	ret = ch348_configure(ch348, port->port_number);
	if (ret) {
		dev_err(&ch348->udev->dev, "Fail to configure err=%d\n", ret);
		return ret;
	}

	return usb_serial_generic_open(tty, port);
}

static int ch348_attach(struct usb_serial *serial)
{
	struct usb_endpoint_descriptor *epcmd, *epstatus;
	struct usb_serial_port *port0 = serial->port[1];
	struct usb_device *usb_dev = serial->dev;
	int status_buffer_size, i, ret;
	struct usb_interface *intf;
	struct ch348 *ch348;

	intf = usb_ifnum_to_if(usb_dev, 0);
	epstatus = &intf->cur_altsetting->endpoint[2].desc;
	epcmd = &intf->cur_altsetting->endpoint[3].desc;

	status_buffer_size = usb_endpoint_maxp(epstatus);

	ch348 = kzalloc(struct_size(ch348, status_buffer, status_buffer_size),
			GFP_KERNEL);
	if (!ch348)
		return -ENOMEM;

	usb_set_serial_data(serial, ch348);

	ch348->udev = serial->dev;
	ch348->serial = serial;
	mutex_init(&ch348->write_lock);

	for (i = 0; i < CH348_MAXPORT; i++)
		init_completion(&ch348->ports[i].write_completion);

	ch348->status_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!ch348->status_urb) {
		ret = -ENOMEM;
		goto err_free_ch348;
	}

	usb_fill_bulk_urb(ch348->status_urb, ch348->udev,
			  usb_rcvbulkpipe(ch348->udev, epstatus->bEndpointAddress),
			  ch348->status_buffer, status_buffer_size,
			  ch348_process_status_urb, ch348);

	ret = usb_submit_urb(ch348->status_urb, GFP_KERNEL);
	if (ret) {
		dev_err(&ch348->udev->dev,
			"%s - failed to submit status/interrupt urb %i\n",
			__func__, ret);
		goto err_free_status_urb;
	}

	ret = usb_serial_generic_submit_read_urbs(port0, GFP_KERNEL);
	if (ret)
		goto err_kill_status_urb;

	ch348->cmd_ep = usb_sndbulkpipe(usb_dev, epcmd->bEndpointAddress);

	return 0;

err_kill_status_urb:
	usb_kill_urb(ch348->status_urb);
err_free_status_urb:
	usb_free_urb(ch348->status_urb);
err_free_ch348:
	kfree(ch348);
	return ret;
}

static void ch348_release(struct usb_serial *serial)
{
	struct ch348 *ch348 = usb_get_serial_data(serial);

	usb_kill_urb(ch348->status_urb);
	usb_free_urb(ch348->status_urb);

	kfree(ch348);
}

static void ch348_print_version(struct usb_serial *serial)
{
	u8 *version_buf;
	int ret;

	version_buf = kzalloc(4, GFP_KERNEL);
	if (!version_buf)
		return;

	ret = usb_control_msg(serial->dev, usb_rcvctrlpipe(serial->dev, 0),
			      CMD_VER,
			      USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_IN,
			      0, 0, version_buf, 4, CH348_CMD_TIMEOUT);
	if (ret < 0)
		dev_dbg(&serial->dev->dev, "Failed to read CMD_VER: %d\n", ret);
	else
		dev_info(&serial->dev->dev, "Found WCH CH348%s\n",
			 (version_buf[1] & 0x80) ? "Q" : "L");

	kfree(version_buf);
}

static int ch348_probe(struct usb_serial *serial, const struct usb_device_id *id)
{
	struct usb_endpoint_descriptor *epread, *epwrite, *epstatus, *epcmd;
	struct usb_device *usb_dev = serial->dev;
	struct usb_interface *intf;
	int ret;

	intf = usb_ifnum_to_if(usb_dev, 0);

	ret = usb_find_common_endpoints(intf->cur_altsetting, &epread, &epwrite,
					NULL, NULL);
	if (ret) {
		dev_err(&serial->dev->dev, "Failed to find basic endpoints ret=%d\n", ret);
		return ret;
	}

	epstatus = &intf->cur_altsetting->endpoint[2].desc;
	if (!usb_endpoint_is_bulk_in(epstatus)) {
		dev_err(&serial->dev->dev, "Missing second bulk in (STATUS/INT)\n");
		return -ENODEV;
	}

	epcmd = &intf->cur_altsetting->endpoint[3].desc;
	if (!usb_endpoint_is_bulk_out(epcmd)) {
		dev_err(&serial->dev->dev, "Missing second bulk out (CMD)\n");
		return -ENODEV;
	}

	ch348_print_version(serial);

	return 0;
}

static int ch348_calc_num_ports(struct usb_serial *serial,
				struct usb_serial_endpoints *epds)
{
	int i;

	for (i = 1; i < CH348_MAXPORT; ++i) {
		epds->bulk_out[i] = epds->bulk_out[0];
		epds->bulk_in[i] = epds->bulk_in[0];
	}

	epds->num_bulk_out = CH348_MAXPORT;
	epds->num_bulk_in = CH348_MAXPORT;

	return CH348_MAXPORT;
}

static int ch348_suspend(struct usb_serial *serial, pm_message_t message)
{
	struct ch348 *ch348 = usb_get_serial_data(serial);

	usb_kill_urb(ch348->status_urb);

	return 0;
}

static int ch348_resume(struct usb_serial *serial)
{
	struct ch348 *ch348 = usb_get_serial_data(serial);
	int ret;

	ret = usb_submit_urb(ch348->status_urb, GFP_KERNEL);
	if (ret) {
		dev_err(&ch348->udev->dev,
			"%s - failed to submit status/interrupt urb %i\n",
			__func__, ret);
		return ret;
	}

	ret = usb_serial_generic_resume(serial);
	if (ret)
		usb_kill_urb(ch348->status_urb);

	return ret;
}

static const struct usb_device_id ch348_ids[] = {
	{ USB_DEVICE(0x1a86, 0x55d9), },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(usb, ch348_ids);

static struct usb_serial_driver ch348_device = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "ch348",
	},
	.id_table =		ch348_ids,
	.num_ports =		CH348_MAXPORT,
	.num_bulk_in =		1,
	.num_bulk_out =		1,
	.open =			ch348_open,
	.set_termios =		ch348_set_termios,
	.process_read_urb =	ch348_process_read_urb,
	.prepare_write_buffer =	ch348_prepare_write_buffer,
	.write =		ch348_write,
	.probe =		ch348_probe,
	.calc_num_ports =	ch348_calc_num_ports,
	.attach =		ch348_attach,
	.release =		ch348_release,
	.suspend =		ch348_suspend,
	.resume =		ch348_resume,
};

static struct usb_serial_driver * const serial_drivers[] = {
	&ch348_device, NULL
};

module_usb_serial_driver(serial_drivers, ch348_ids);

MODULE_AUTHOR("Corentin Labbe <clabbe@baylibre.com>");
MODULE_DESCRIPTION("USB CH348 Octo port serial converter driver");
MODULE_LICENSE("GPL");
