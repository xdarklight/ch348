// SPDX-License-Identifier: GPL-2.0
/*
 * USB serial driver for USB to Octal UARTs chip ch348.
 *
 * Copyright (C) 2023 Corentin Labbe <clabbe@baylibre.com>
 * With the help of Neil Armstrong <neil.armstrong@linaro.org>
 * Copyright (C) 2025 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * Based on the ch9344 driver:
 *   https://github.com/WCHSoftGroup/ch9344ser_linux/
 *   Copyright (C) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
 */

#include <linux/bitmap.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/kfifo.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/serial.h>
#include <linux/serial_reg.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/usb.h>
#include <linux/usb/serial.h>

#define CH348_CMD_TIMEOUT		2000

#define CMD_W_R				0xc0
#define CMD_W_BR			0x80

#define CMD_WB_E			0x90
#define CMD_RB_E			0xc0

/* R_C1 = 0x01 is UART_IER compatible */

/* no official documentation available for R_C2 */
#define R_C2				0x02
#define R_C2_ACTIVATE			0x87

/* no official documentation available for R_C3 */
#define R_C3				0x03
#define R_C3_BREAK_OFF			0x60
#define R_C3_BREAK_ON			0x61

/* no official documentation available for R_C4 */
#define R_C4				0x04
#define R_C4_DTR_OFF			0x00
#define R_C4_DTR_ON			0x01
#define R_C4_RTS_OFF			0x10
#define R_C4_RTS_ON			0x11
#define R_C4_ACTIVATE			0x08
#define R_C4_HW_FLOW			0x50
#define R_C4_NO_RTS			0x51

#define R_C5				0x06

#define VEN_R				0x85
#define VEN_R_MODEM_STATUS_CHANGE	0x06

#define VEN_W				0x8a

#define CMD_VER				0x96

#define R_MOD				0x97
#define R_IO_D				0x98
#define R_IO_O				0x99
#define R_IO_I				0x9b
#define R_TM_O				0x9c
#define R_INIT				0xa1

/*
 * The CH348 multiplexes rx & tx into a pair of bulk USB endpoints for the 8
 * serial ports, and another pair of bulk USB endpoints to set port settings
 * and receive port status events.
 *
 * For serial TX we need to not only wait for the URB to complete but also for
 * the UART_IIR_THRI signal (otherwise we overwrite the transfer-in-progress
 * buffer of that port). Thus we manage port->write_urbs_free so that USB
 * serial core cannot submit more URBs until we receive UART_IIR_THRI.
 *
 * The config bulk write endpoint is relocated from it's actual endpoint 1 to
 * an additional endpoint/port after the 8 standard serial ports. This port is
 * not visible to userspace (it's only used for managing the config).
 *
 * For bulk reads we use USB serial core's helpers for both, serial RX and
 * status/int handling as it simplifies our code.
 */
#define CH348_MAXPORT				8
#define CH348_PORTNUM_SERIAL_RX			0
#define CH348_PORTNUM_STATUS_INT		1
#define CH348_PORTNUM_CONFIG_WRITE		CH348_MAXPORT

#define CH348_RX_PORT_MAX_LENGTH		30

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

#define CH348_TX_HDRSIZE	offsetof(struct ch348_txbuf, data)

enum ch348_package {
	CH348Q, /* LQFP48 (small) */
	CH348L, /* LQFP100 (large) */
};

/**
 * struct ch348 - main container for all this driver information
 * @serial:		pointer to the serial structure
 * @open_ports:		bitmap of ports that are currently opened
 * @open_ports_lock:	protect against concurrent modification of open_ports
 * @package_type:	indicates package type
 */
struct ch348 {
	struct usb_serial *serial;

	DECLARE_BITMAP(open_ports, CH348_MAXPORT);
	struct mutex open_ports_lock;

	enum ch348_package package_type;
};

struct ch348_config_buf {
	u8 cmd;
	u8 reg;
	u8 data[];
} __packed;

struct ch348_config_data_init {
	u8 port;
	__be32 baudrate;
	u8 format;
	u8 paritytype;
	u8 databits;
	u8 rate;
	u8 unknown;
} __packed;

#define CH348_CONFIG_DATA_INIT_FORMAT_ONE_STOPBIT	0x2
#define CH348_CONFIG_DATA_INIT_FORMAT_TWO_STOPBITS	0x0

struct ch348_status_entry {
	u8 portnum;
	u8 reg_iir;
	union {
		u8 unknown8;
		u8 unknown16[2];
		u8 lsr_signal;
		u8 modem_signal;
		struct ch348_config_data_init init_data;
	} data;
} __packed;

#define CH348_STATUS_ENTRY_PORTNUM_MASK			0xf

static void ch348_kill_port_read_urbs(struct usb_serial_port *port)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(port->read_urbs); ++i)
		usb_kill_urb(port->read_urbs[i]);
}

static int ch348_submit_read_urbs(struct usb_serial *serial, gfp_t mem_flags)
{
	struct usb_serial_port *rx_port, *status_int_port;
	int ret = 0;

	rx_port = serial->port[CH348_PORTNUM_SERIAL_RX];
	ret = usb_serial_generic_submit_read_urbs(rx_port, mem_flags);
	if (ret) {
		dev_err(&serial->dev->dev,
			"Failed to submit read URBs of RX port: %d\n", ret);
		return ret;
	}

	status_int_port = serial->port[CH348_PORTNUM_STATUS_INT];
	ret = usb_serial_generic_submit_read_urbs(status_int_port, mem_flags);
	if (ret) {
		dev_err(&serial->dev->dev,
			"Failed to submit read URBs of STATUS/INT port: %d\n",
			ret);
		ch348_kill_port_read_urbs(rx_port);
		return ret;
	}

	return 0;
}

static void ch348_kill_read_urbs(struct usb_serial *serial)
{
	ch348_kill_port_read_urbs(serial->port[CH348_PORTNUM_STATUS_INT]);
	ch348_kill_port_read_urbs(serial->port[CH348_PORTNUM_SERIAL_RX]);
}

static void ch348_tx_done(struct usb_serial_port *port)
{
	int tx_bytes;

	scoped_guard(spinlock_irqsave, &port->lock)
		tx_bytes = port->tx_bytes;

	/*
	 * Only continue processing in USB serial core if it knows about the
	 * data (and it's not a spurious done status on port open).
	 */
	if (tx_bytes > CH348_TX_HDRSIZE) {
		port->icount.tx += tx_bytes - CH348_TX_HDRSIZE;
		usb_serial_generic_write_bulk_callback(port->write_urb);
	}
}

static void ch348_process_status_urb(struct usb_serial *serial, struct urb *urb)
{
	struct ch348 *ch348 = usb_get_serial_data(serial);
	struct ch348_status_entry *status_entry;
	struct usb_serial_port *port;
	unsigned int i, status_len;
	u8 portnum;

	if (urb->actual_length < 3) {
		dev_dbg_ratelimited(&ch348->serial->dev->dev,
				    "Received too short status buffer with %u bytes\n",
				    urb->actual_length);
		return;
	}

	for (i = 0; i < urb->actual_length;) {
		status_entry = urb->transfer_buffer + i;
		portnum = status_entry->portnum & CH348_STATUS_ENTRY_PORTNUM_MASK;

		if (portnum >= CH348_MAXPORT) {
			dev_dbg_ratelimited(&ch348->serial->dev->dev,
					    "Invalid port %d in status entry\n",
					    portnum);
			break;
		}

		port = serial->port[portnum];
		status_len = sizeof(*status_entry) - sizeof(status_entry->data);

		if (!status_entry->reg_iir) {
			status_len += sizeof(status_entry->data.unknown8);
			dev_dbg(&port->dev, "Ignoring status with zero reg_iir\n");
		} else if (status_entry->reg_iir == R_INIT) {
			status_len += sizeof(status_entry->data.init_data);
		} else if (status_entry->reg_iir == VEN_R) {
			status_len += sizeof(status_entry->data.unknown16);
		} else if ((status_entry->reg_iir & UART_IIR_ID) == UART_IIR_RLSI) {
			status_len += sizeof(status_entry->data.lsr_signal);

			if (status_entry->data.lsr_signal & UART_LSR_OE)
				port->icount.overrun++;
			if (status_entry->data.lsr_signal & UART_LSR_PE)
				port->icount.parity++;
			if (status_entry->data.lsr_signal & UART_LSR_FE)
				port->icount.frame++;
			if (status_entry->data.lsr_signal & UART_LSR_BI)
				port->icount.brk++;
		} else if ((status_entry->reg_iir & UART_IIR_ID) == UART_IIR_THRI) {
			status_len += sizeof(status_entry->data.unknown8);
			ch348_tx_done(port);
		} else {
			status_len += sizeof(status_entry->data.unknown8);
			dev_dbg_ratelimited(&port->dev,
					    "Unsupported status with reg_iir 0x%02x\n",
					    status_entry->reg_iir);
		}

		i += status_len;
	}
}

static void ch348_process_serial_rx_urb(struct usb_serial *serial,
					struct urb *urb)
{
	unsigned int portnum, serial_rx_len, i;
	struct usb_serial_port *port;
	struct ch348_rxbuf *rxb;

	if (urb->actual_length < 2) {
		dev_dbg(&serial->dev->dev, "Empty rx buffer\n");
		return;
	}

	for (i = 0; i < urb->actual_length; i += sizeof(*rxb)) {
		rxb = urb->transfer_buffer + i;
		portnum = rxb->port;
		if (portnum >= CH348_MAXPORT) {
			dev_dbg(&serial->dev->dev, "Invalid port %d\n", portnum);
			break;
		}

		port = serial->port[portnum];

		serial_rx_len = rxb->length;
		if (serial_rx_len > CH348_RX_PORT_MAX_LENGTH) {
			dev_dbg(&port->dev, "Invalid length %d for port %d\n",
				serial_rx_len, portnum);
			break;
		}

		tty_insert_flip_string(&port->port, rxb->data, serial_rx_len);
		tty_flip_buffer_push(&port->port);

		port->icount.rx += serial_rx_len;
	}
}

static void ch348_process_read_urb(struct urb *urb)
{
	struct usb_serial_port *port = urb->context;

	if (port->port_number == CH348_PORTNUM_SERIAL_RX)
		ch348_process_serial_rx_urb(port->serial, urb);
	else if (port->port_number == CH348_PORTNUM_STATUS_INT)
		ch348_process_status_urb(port->serial, urb);
}

static void ch348_write_bulk_callback(struct urb *urb)
{
	if (urb->status)
		usb_serial_generic_write_bulk_callback(urb);
	/* else: processing continues once we receive UART_IIR_THRI */
}

static int ch348_write_config(struct ch348 *ch348, u8 cmd, u8 reg, void *data,
			      size_t len)
{
	struct usb_serial_port *config_port;
	struct ch348_config_buf *buf;
	int config_pipe, ret;
	size_t buf_len;

	buf_len = struct_size(buf, data, len);

	buf = kzalloc(buf_len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	buf->cmd = cmd;
	buf->reg = reg;

	if (len)
		memcpy(buf->data, data, len);

	config_port = ch348->serial->port[CH348_PORTNUM_CONFIG_WRITE];
	config_pipe = usb_sndbulkpipe(ch348->serial->dev,
				      config_port->bulk_out_endpointAddress);

	ret = usb_bulk_msg(ch348->serial->dev, config_pipe, buf, buf_len, NULL,
			   CH348_CMD_TIMEOUT);

	kfree(buf);

	return ret < 0 ? ret : 0;
}

static int ch348_port_config(struct usb_serial_port *port, u8 cmd, u8 reg,
			     u8 control)
{
	struct ch348 *ch348 = usb_get_serial_data(port->serial);
	int ret;

	if (port->port_number < 4)
		reg += 0x10 * port->port_number;
	else
		reg += 0x10 * (port->port_number - 4) + 0x08;

	ret = ch348_write_config(ch348, cmd, reg, &control, sizeof(control));
	if (ret < 0)
		dev_err(&port->dev,
			"Failed to write port config: %d\n", ret);

	return ret;
}

static int ch348_prepare_write_buffer(struct usb_serial_port *port, void *dest,
				      size_t size)
{
	struct ch348_txbuf *rxt = dest;
	int count;

	count = kfifo_out_locked(&port->write_fifo, rxt->data,
				 size - CH348_TX_HDRSIZE, &port->lock);

	rxt->port = port->port_number;
	rxt->length = cpu_to_le16(count);

	return count + CH348_TX_HDRSIZE;
}

static void ch348_set_termios(struct tty_struct *tty, struct usb_serial_port *port,
			      const struct ktermios *termios_old)
{
	struct ch348 *ch348 = usb_get_serial_data(port->serial);
	struct ch348_config_data_init config = {};
	struct ktermios *termios = &tty->termios;
	int ret, portnum = port->port_number;
	speed_t	baudrate;

	if (termios_old && !tty_termios_hw_change(&tty->termios, termios_old))
		return;

	/*
	 * The datasheet states that only baud rates in range of 1200..6000000
	 * are supported. Tests with an oscilloscope confirm that even when
	 * configuring a baud rate slower than 1200 the output stays at around
	 * 1200 baud.
	 */
	baudrate = clamp(tty_get_baud_rate(tty), 1200, 6000000);
	tty_termios_encode_baud_rate(&tty->termios, baudrate, baudrate);

	if (termios->c_cflag & PARENB) {
		if  (termios->c_cflag & CMSPAR) {
			if (termios->c_cflag & PARODD)
				config.paritytype = 3;
			else
				config.paritytype = 4;
		} else {
			if (termios->c_cflag & PARODD)
				config.paritytype = 1;
			else
				config.paritytype = 2;
		}
	} else {
		config.paritytype = 0;
	}

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		config.databits = 5;
		break;
	case CS6:
		config.databits = 6;
		break;
	case CS7:
		config.databits = 7;
		break;
	case CS8:
	default:
		config.databits = 8;
		break;
	}

	config.port = portnum;
	config.baudrate = cpu_to_be32(baudrate);

	if (termios->c_cflag & CSTOPB)
		config.format = CH348_CONFIG_DATA_INIT_FORMAT_ONE_STOPBIT;
	else
		config.format = CH348_CONFIG_DATA_INIT_FORMAT_TWO_STOPBITS;

	config.rate = max_t(speed_t, 5, (10000 * 15 / baudrate) + 1);

	ret = ch348_write_config(ch348, CMD_WB_E | portnum, R_INIT, &config,
				 sizeof(config));
	if (ret < 0) {
		dev_err(&port->dev, "Failed to change line settings: %d\n",
			ret);
		if (termios_old)
			tty->termios = *termios_old;
	}

	ch348_port_config(port, CMD_W_R, UART_IER, UART_IER_RDI |
			  UART_IER_THRI | UART_IER_RLSI | UART_IER_MSI);
}

static int ch348_break_ctl(struct tty_struct *tty, int on)
{
	struct usb_serial_port *port = tty->driver_data;

	return ch348_port_config(port, CMD_W_BR, R_C3,
				 on ? R_C3_BREAK_ON : R_C3_BREAK_OFF);
}

static void ch348_dtr_rts(struct usb_serial_port *port, int on)
{
	struct ch348 *ch348 = usb_get_serial_data(port->serial);
	int ret;

	/*
	 * Only the first four ports have the modem control pins routed outside
	 * the CH348Q package.
	 */
	if (ch348->package_type == CH348Q && port->port_number >= 4) {
		dev_dbg(&port->dev,
			"DTR/RTS is not supported on CH348Q port %u\n",
			port->port_number);
		return;
	}

	ret = ch348_port_config(port, CMD_W_BR, R_C4,
				on ? R_C4_DTR_ON : R_C4_DTR_OFF);
	if (ret)
		dev_err(&port->dev, "Failed to %s DTR in R_C4: %d\n",
			str_enable_disable(!!on), ret);

	ret = ch348_port_config(port, CMD_W_BR, R_C4,
				on ? R_C4_RTS_ON : R_C4_RTS_OFF);
	if (ret)
		dev_err(&port->dev, "Failed to %s RTS in R_C4: %d\n",
			str_enable_disable(!!on), ret);

	ret = ch348_port_config(port, CMD_WB_E, VEN_R,
				VEN_R_MODEM_STATUS_CHANGE);
	if (ret)
		dev_err(&port->dev, "Failed to write VEN_R: %d\n", ret);
}

static bool ch348_tx_empty(struct usb_serial_port *port)
{
	scoped_guard(spinlock_irqsave, &port->lock)
		return !port->tx_bytes;
}

static int ch348_port_probe(struct usb_serial_port *port)
{
	unsigned int i;

	/*
	 * Reserve all but the first write URB to prevent further URB
	 * submissions until UART_IIR_THRI is received. Otherwise we overwrite
	 * the transfer-in-progress buffer on the chip.
	 */
	for (i = 1; i < ARRAY_SIZE(port->write_urbs); ++i)
		clear_bit(i, &port->write_urbs_free);

	return 0;
}

static int ch348_open(struct tty_struct *tty, struct usb_serial_port *port)
{
	struct ch348 *ch348 = usb_get_serial_data(port->serial);
	int ret;

	scoped_guard(mutex, &ch348->open_ports_lock) {
		if (bitmap_empty(ch348->open_ports, CH348_MAXPORT)) {
			ret = ch348_submit_read_urbs(port->serial, GFP_KERNEL);
			if (ret)
				return ret;
		}

		set_bit(port->port_number, ch348->open_ports);
	}

	if (tty)
		ch348_set_termios(tty, port, NULL);

	ret = ch348_port_config(port, CMD_W_R, R_C2, R_C2_ACTIVATE);
	if (ret) {
		dev_err(&port->dev, "Failed to configure R_C2_ACTIVATE: %d\n",
			ret);
		goto err_kill_read_urbs;
	}

	ret = ch348_port_config(port, CMD_W_R, R_C4, R_C4_ACTIVATE);
	if (ret) {
		dev_err(&port->dev, "Failed to configure R_C4_ACTIVATE: %d\n",
			ret);
		goto err_kill_read_urbs;
	}

	return 0;

err_kill_read_urbs:
	scoped_guard(mutex, &ch348->open_ports_lock) {
		clear_bit(port->port_number, ch348->open_ports);

		if (bitmap_empty(ch348->open_ports, CH348_MAXPORT))
			ch348_kill_read_urbs(port->serial);
	}
	return ret;
}

static void ch348_close(struct usb_serial_port *port)
{
	struct ch348 *ch348 = usb_get_serial_data(port->serial);

	usb_kill_urb(port->write_urb);

	scoped_guard(spinlock_irqsave, &port->lock) {
		kfifo_reset_out(&port->write_fifo);
		set_bit(0, &port->write_urbs_free);
		port->tx_bytes = 0;
	}

	scoped_guard(mutex, &ch348->open_ports_lock) {
		clear_bit(port->port_number, ch348->open_ports);

		if (bitmap_empty(ch348->open_ports, CH348_MAXPORT))
			ch348_kill_read_urbs(port->serial);
	}
}

static int ch348_detect_version(struct usb_serial *serial)
{
	struct ch348 *ch348 = usb_get_serial_data(serial);
	u8 version_buf[4];
	int ret;

	ret = usb_control_msg_recv(serial->dev, 0, CMD_VER,
				   USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_IN,
				   0, 0, version_buf, sizeof(version_buf),
				   CH348_CMD_TIMEOUT, GFP_KERNEL);
	if (ret) {
		dev_err(&serial->dev->dev, "Failed to read CMD_VER: %d\n", ret);
		return ret;
	}

	ch348->package_type = (version_buf[1] & 0x80) ? CH348Q : CH348L;

	dev_info(&serial->dev->dev, "Found WCH CH348%c version 0x%02x\n",
		 ch348->package_type == CH348Q ? 'Q' : 'L', version_buf[0]);

	return 0;
}

static int ch348_attach(struct usb_serial *serial)
{
	struct ch348 *ch348;
	int ret;

	ch348 = kzalloc(sizeof(*ch348), GFP_KERNEL);
	if (!ch348)
		return -ENOMEM;

	usb_set_serial_data(serial, ch348);

	ch348->serial = serial;

	mutex_init(&ch348->open_ports_lock);

	ret = ch348_detect_version(serial);
	if (ret)
		goto err_free_ch348;

	return 0;

err_free_ch348:
	kfree(ch348);
	return ret;
}

static void ch348_release(struct usb_serial *serial)
{
	struct ch348 *ch348 = usb_get_serial_data(serial);

	mutex_destroy(&ch348->open_ports_lock);

	kfree(ch348);
}

static int ch348_calc_num_ports(struct usb_serial *serial,
				struct usb_serial_endpoints *epds)
{
	unsigned int i;

	/*
	 * Reserve a bulk out for each serial port plus an additional one
	 * (which is not registered as a serial port) for the config bulk out.
	 */
	epds->num_bulk_out = CH348_PORTNUM_CONFIG_WRITE + 1;

	epds->bulk_out[CH348_PORTNUM_CONFIG_WRITE] = epds->bulk_out[1];

	/* Use the serial data bulk out endpoint for each port */
	for (i = 0; i < CH348_MAXPORT; ++i)
		epds->bulk_out[i] = epds->bulk_out[0];

	return CH348_MAXPORT;
}

static int ch348_resume(struct usb_serial *serial)
{
	struct ch348 *ch348 = usb_get_serial_data(serial);
	unsigned int i;
	int ret;

	scoped_guard(mutex, &ch348->open_ports_lock) {
		if (bitmap_empty(ch348->open_ports, CH348_MAXPORT))
			return 0;

		ret = ch348_submit_read_urbs(serial, GFP_NOIO);
		if (ret)
			return ret;

		for_each_set_bit(i, ch348->open_ports, CH348_MAXPORT) {
			ret = usb_serial_generic_write_start(serial->port[i],
							     GFP_NOIO);
			if (ret)
				return ret;
		}
	}

	return 0;
}

static const struct usb_device_id ch348_ids[] = {
	{ USB_DEVICE(0x1a86, 0x55d9) },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(usb, ch348_ids);

static struct usb_serial_driver ch348_device = {
	.driver = {
		.name = "ch348",
	},
	.id_table =		ch348_ids,
	.num_ports =		CH348_MAXPORT,
	.num_bulk_in =		2,
	.num_bulk_out =		2,
	.port_probe =		ch348_port_probe,
	.open =			ch348_open,
	.close =		ch348_close,
	.set_termios =		ch348_set_termios,
	.break_ctl =		ch348_break_ctl,
	.dtr_rts =		ch348_dtr_rts,
	.tx_empty =		ch348_tx_empty,
	.process_read_urb =	ch348_process_read_urb,
	.write_bulk_callback =	ch348_write_bulk_callback,
	.prepare_write_buffer =	ch348_prepare_write_buffer,
	.get_icount =		usb_serial_generic_get_icount,
	.calc_num_ports =	ch348_calc_num_ports,
	.attach =		ch348_attach,
	.release =		ch348_release,
	.resume =		ch348_resume,
};

static struct usb_serial_driver * const serial_drivers[] = {
	&ch348_device, NULL
};

module_usb_serial_driver(serial_drivers, ch348_ids);

MODULE_AUTHOR("Corentin Labbe <clabbe@baylibre.com>");
MODULE_AUTHOR("Martin Blumenstingl <martin.blumenstingl@googlemail.com>");
MODULE_DESCRIPTION("USB CH348 Octo port serial converter driver");
MODULE_LICENSE("GPL");
