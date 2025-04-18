// SPDX-License-Identifier: GPL-2.0
/*
 * USB serial driver for USB to Octal UARTs chip ch348.
 *
 * Copyright (C) 2023 Corentin Labbe <clabbe@baylibre.com>
 * With the help of Neil Armstrong <neil.armstrong@linaro.org>
 * Copyright (C) 2024 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * Based on the ch9344 driver:
 *   https://github.com/WCHSoftGroup/ch9344ser_linux/
 *   Copyright (C) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 */

#include <linux/completion.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kfifo.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/overflow.h>
#include <linux/serial.h>
#include <linux/serial_reg.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/usb.h>
#include <linux/usb/serial.h>
#include <linux/workqueue.h>

#define CH348_CMD_TIMEOUT   2000

#define CH348_CTO_D	0x01
#define CH348_CTO_R	0x02

#define CH348_CTI_C	0x10
#define CH348_CTI_DSR	0x20
#define CH348_CTI_R	0x40
#define CH348_CTI_DCD	0x80

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

#define CMD_VER		0x96

/* 0x10 is normally UART_MCR_LOOP but for CH348 it's UART_MCR_RTS */
#define UART_MCR_RTS_CH348	0x10

/*
 * The CH348 multiplexes rx & tx into a pair of Bulk USB endpoints for the 8
 * serial ports, and another pair of Bulk USB endpoints to set port settings
 * and receive port status events.
 *
 * The USB serial cores ties every Bulk endpoints pairs to each ports, In our
 * case it will set port 0 with the rx/tx endpoints and port 1 with the
 * setup/status endpoints.
 *
 * For bulk writes we skip all of USB serial core's helpers and implement it on
 * our own since for serial TX we need to not only wait for the URB to complete
 * but also for the UART_IIR_THRI signal.
 *
 * For bulk reads we use USB serial core's helpers, even for the status/int
 * handling as it simplifies our code.
 */
#define CH348_MAXPORT				8
#define CH348_PORTNUM_SERIAL_RX_TX		0
#define CH348_PORTNUM_STATUS_INT_CONFIG		1

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

/*
 * struct ch348_port - per-port information
 * @uartmode:		UART port current mode
 * @baudrate:		A cached copy of current baudrate for the RX logic
 */
struct ch348_port {
	u8 uartmode;
	speed_t baudrate;
};

/*
 * struct ch348 - main container for all this driver information
 * @ports:		List of per-port information
 * @serial:		pointer to the serial structure
 * @write_work:		worker for processing the write queues
 * @txbuf_completion:	indicates that the TX buffer has been fully written out
 * @tx_ep:		endpoint number for serial data transmit/write operation
 * @config_ep:		endpoint number for configure operations
 * @small_package:	indicates package size: small (CH348Q) or large (CH348L)
 */
struct ch348 {
	struct ch348_port ports[CH348_MAXPORT];
	struct usb_serial *serial;

	struct work_struct write_work;
	struct completion txbuf_completion;

	int tx_ep;
	int config_ep;

	bool small_package;
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

#define CH348_CONFIG_DATA_INIT_FORMAT_STOPBITS		0x2
#define CH348_CONFIG_DATA_INIT_FORMAT_NO_STOPBITS	0x0

struct ch348_status_entry {
	u8 portnum;
	u8 reg_iir;
	union {
		u8 unknown;
		u8 lsr_signal;
		u8 modem_signal;
		struct ch348_config_data_init init_data;
	} data;
} __packed;

#define CH348_STATUS_ENTRY_PORTNUM_MASK		0xf

static void ch348_process_status_urb(struct usb_serial *serial, struct urb *urb)
{
	struct ch348 *ch348 = usb_get_serial_data(serial);
	struct ch348_status_entry *status_entry;
	struct usb_serial_port *port;
	unsigned int i, status_len;
	u8 portnum;

	if (urb->actual_length < 3) {
		dev_warn_ratelimited(&ch348->serial->dev->dev,
				     "Received too short status buffer with %u bytes\n",
				     urb->actual_length);
		return;
	}

	for (i = 0; i < urb->actual_length;) {
		status_entry = urb->transfer_buffer + i;
		portnum = status_entry->portnum & CH348_STATUS_ENTRY_PORTNUM_MASK;

		if (portnum >= CH348_MAXPORT) {
			dev_warn_ratelimited(&ch348->serial->dev->dev,
					     "Invalid port %d in status entry\n",
					     portnum);
			break;
		}

		port = serial->port[portnum];
		status_len = sizeof(*status_entry) - sizeof(status_entry->data);

		if (!status_entry->reg_iir) {
			status_len += sizeof(status_entry->data.unknown);
			dev_dbg(&port->dev, "Ignoring status with zero reg_iir\n");
		} else if (status_entry->reg_iir == R_INIT) {
			status_len += sizeof(status_entry->data.init_data);
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
			status_len += sizeof(status_entry->data.unknown);
			complete_all(&ch348->txbuf_completion);
		} else {
			status_len += sizeof(status_entry->data.unknown);
			dev_warn_ratelimited(&port->dev,
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

	if (port->port_number == CH348_PORTNUM_SERIAL_RX_TX)
		ch348_process_serial_rx_urb(port->serial, urb);
	else if (port->port_number == CH348_PORTNUM_STATUS_INT_CONFIG)
		ch348_process_status_urb(port->serial, urb);
	else
		dev_warn_ratelimited(&port->serial->dev->dev,
				     "Ignoring read URB callback for unknown port/endpoint %u\n",
				     port->port_number);
}

static int ch348_write_config(struct ch348 *ch348, u8 cmd, u8 reg, void *data,
			      size_t len)
{
	struct ch348_config_buf *buf;
	size_t buf_len;
	int ret;

	buf_len = struct_size(buf, data, len);

	buf = kzalloc(buf_len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	buf->cmd = cmd;
	buf->reg = reg;

	if (len)
		memcpy(buf->data, data, len);

	ret = usb_bulk_msg(ch348->serial->dev, ch348->config_ep, buf, buf_len,
			   NULL, CH348_CMD_TIMEOUT);

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
		dev_err(&ch348->serial->dev->dev,
			"Failed to write port config: %d\n", ret);

	return ret;
}

static int ch348_write(struct tty_struct *tty, struct usb_serial_port *port,
		       const unsigned char *buf, int count)
{
	struct ch348 *ch348 = usb_get_serial_data(port->serial);

	if (!count)
		return 0;

	count = kfifo_in_locked(&port->write_fifo, buf, count, &port->lock);

	schedule_work(&ch348->write_work);

	return count;
}

static int ch348_set_uartmode(struct usb_serial_port *port, u8 mode)
{
	struct ch348 *ch348 = usb_get_serial_data(port->serial);
	unsigned int portnum = port->port_number;
	int ret;

	if (ch348->ports[portnum].uartmode == M_NOR && mode == M_HF) {
		ret = ch348_port_config(port, CMD_W_BR, UART_MCR,
					UART_MCR_DTR | UART_MCR_RTS_CH348 |
					UART_MCR_TCRTLR);
		if (ret)
			return ret;
		ch348->ports[portnum].uartmode = M_HF;
	}

	if (ch348->ports[portnum].uartmode == M_HF && mode == M_NOR) {
		ret = ch348_port_config(port, CMD_W_BR, UART_MCR,
					UART_MCR_RTS_CH348 | UART_MCR_TCRTLR);
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
	ch348->ports[port->port_number].baudrate = baudrate;

	if (termios->c_cflag & PARENB) {
		if  (termios->c_cflag & CMSPAR) {
			if (termios->c_cflag & PARODD)
				config.paritytype = 3;
			else
				config.paritytype = 4;
		} else if (termios->c_cflag & PARODD) {
			config.paritytype = 1;
		} else {
			config.paritytype = 2;
		}
	} else {
		config.paritytype = 0;
	}

	switch (C_CSIZE(tty)) {
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
		config.format = CH348_CONFIG_DATA_INIT_FORMAT_STOPBITS;
	else
		config.format = CH348_CONFIG_DATA_INIT_FORMAT_NO_STOPBITS;

	config.rate = max_t(speed_t, 5, (10000 * 15 / baudrate) + 1);

	ret = ch348_write_config(ch348, CMD_WB_E | portnum, R_INIT, &config,
				 sizeof(config));
	if (ret < 0) {
		dev_err(&ch348->serial->dev->dev,
			"Failed to change line settings: err=%d\n", ret);
		goto out;
	}

	ret = ch348_port_config(port, CMD_W_R, UART_IER, UART_IER_RDI |
				UART_IER_THRI | UART_IER_RLSI | UART_IER_MSI);
	if (ret < 0)
		goto out;

	if (C_CRTSCTS(tty))
		ret = ch348_set_uartmode(port, M_HF);
	else
		ret = ch348_set_uartmode(port, M_NOR);

out:
	if (ret && termios_old)
		tty->termios = *termios_old;
}

static int ch348_open(struct tty_struct *tty, struct usb_serial_port *port)
{
	int ret;

	clear_bit(USB_SERIAL_THROTTLED, &port->flags);

	if (tty)
		ch348_set_termios(tty, port, NULL);

	ret = ch348_port_config(port, CMD_W_R, UART_FCR,
				UART_FCR_ENABLE_FIFO | UART_FCR_CLEAR_RCVR |
				UART_FCR_CLEAR_XMIT | UART_FCR_T_TRIG_00 |
				UART_FCR_R_TRIG_10);
	if (ret) {
		dev_err(&port->serial->dev->dev,
			"Failed to configure UART_FCR, err=%d\n", ret);
		return ret;
	}

	ret = ch348_port_config(port, CMD_W_BR, UART_MCR, UART_MCR_OUT2);
	if (ret) {
		dev_err(&port->serial->dev->dev,
			"Failed to configure UART_MCR, err=%d\n", ret);
		return ret;
	}

	return 0;
}

static void ch348_close(struct usb_serial_port *port)
{
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
	kfifo_reset_out(&port->write_fifo);
	spin_unlock_irqrestore(&port->lock, flags);
}

static void ch348_write_work(struct work_struct *work)
{
	struct ch348 *ch348 = container_of(work, struct ch348, write_work);
	struct usb_serial_port *port, *hw_tx_port;
	unsigned int i, max_bytes;
	struct ch348_txbuf *rxt;
	unsigned long flags;
	int ret, count;

	reinit_completion(&ch348->txbuf_completion);

	hw_tx_port = ch348->serial->port[CH348_PORTNUM_SERIAL_RX_TX];
	rxt = hw_tx_port->write_urbs[0]->transfer_buffer;

	for (i = 0; i < CH348_MAXPORT; i++) {
		port = ch348->serial->port[i];

		if (ch348->ports[i].baudrate < 9600)
			/*
			 * Writing larger buffers can take longer than the
			 * hardware allows before discarding the write buffer.
			 * Limit the transfer size in such cases.
			 * These values have been found by empirical testing.
			 */
			max_bytes = 128;
		else
			/*
			 * Only ingest as many bytes as we can transfer with
			 * one URB at a time keeping the TX header in mind.
			 */
			max_bytes = hw_tx_port->bulk_out_size - CH348_TX_HDRSIZE;

		count = kfifo_out_locked(&port->write_fifo, rxt->data,
					 max_bytes, &port->lock);
		if (count)
			break;
	}

	if (!count)
		return;

	spin_lock_irqsave(&port->lock, flags);
	port->tx_bytes += count;
	spin_unlock_irqrestore(&port->lock, flags);

	rxt->port = port->port_number;
	rxt->length = cpu_to_le16(count);

	usb_serial_debug_data(&port->dev, __func__, count + CH348_TX_HDRSIZE,
			      (const unsigned char *)rxt);

	ret = usb_bulk_msg(ch348->serial->dev, ch348->tx_ep, rxt,
			   count + CH348_TX_HDRSIZE, NULL, CH348_CMD_TIMEOUT);
	if (ret) {
		dev_err_console(port,
				"Failed to bulk write TX buffer, err=%d\n",
				ret);
		goto write_done;
	}

	if (!wait_for_completion_timeout(&ch348->txbuf_completion,
					 msecs_to_jiffies(CH348_CMD_TIMEOUT)))
		dev_err_console(port,
				"Failed to wait for TX buffer to be fully written out\n");

write_done:
	spin_lock_irqsave(&port->lock, flags);
	port->tx_bytes -= count;
	spin_unlock_irqrestore(&port->lock, flags);

	port->icount.tx += count;

	schedule_work(&ch348->write_work);
	usb_serial_port_softint(port);
}

static int ch348_submit_urbs(struct usb_serial *serial)
{
	int ret;

	ret = usb_serial_generic_open(NULL,
				      serial->port[CH348_PORTNUM_SERIAL_RX_TX]);
	if (ret) {
		dev_err(&serial->dev->dev,
			"Failed to open RX/TX port, err=%d\n", ret);
		return ret;
	}

	ret = usb_serial_generic_open(NULL,
				      serial->port[CH348_PORTNUM_STATUS_INT_CONFIG]);
	if (ret) {
		dev_err(&serial->dev->dev,
			"Failed to submit STATUS/INT URB, err=%d\n", ret);
		usb_serial_generic_close(serial->port[CH348_PORTNUM_SERIAL_RX_TX]);
	}

	return ret;
}

static void ch348_kill_urbs(struct usb_serial *serial)
{
	usb_serial_generic_close(serial->port[CH348_PORTNUM_STATUS_INT_CONFIG]);
	usb_serial_generic_close(serial->port[CH348_PORTNUM_SERIAL_RX_TX]);
}

static int ch348_detect_version(struct usb_serial *serial)
{
	struct ch348 *ch348 = usb_get_serial_data(serial);
	u8 *version_buf;
	int ret;

	version_buf = kzalloc(4, GFP_KERNEL);
	if (!version_buf)
		return -ENOMEM;

	ret = usb_control_msg(serial->dev, usb_rcvctrlpipe(serial->dev, 0),
			      CMD_VER,
			      USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_IN,
			      0, 0, version_buf, 4, CH348_CMD_TIMEOUT);
	if (ret < 0) {
		dev_err(&serial->dev->dev, "Failed to read CMD_VER: %d\n", ret);
		goto out;
	}

	ret = 0;
	ch348->small_package = !!(version_buf[1] & 0x80);

	dev_info(&serial->dev->dev, "Found WCH CH348%c version 0x%02x\n",
		 ch348->small_package ? 'Q' : 'L', version_buf[0]);

out:
	kfree(version_buf);

	return ret;
}

static int ch348_attach(struct usb_serial *serial)
{
	struct usb_serial_port *tx_port, *config_port;
	struct ch348 *ch348;
	int ret;

	ch348 = kzalloc(sizeof(*ch348), GFP_KERNEL);
	if (!ch348)
		return -ENOMEM;

	usb_set_serial_data(serial, ch348);

	ch348->serial = serial;

	INIT_WORK(&ch348->write_work, ch348_write_work);

	init_completion(&ch348->txbuf_completion);

	tx_port = ch348->serial->port[CH348_PORTNUM_SERIAL_RX_TX];
	ch348->tx_ep = usb_sndbulkpipe(serial->dev,
				       tx_port->bulk_out_endpointAddress);

	config_port = ch348->serial->port[CH348_PORTNUM_STATUS_INT_CONFIG];
	ch348->config_ep = usb_sndbulkpipe(serial->dev,
					   config_port->bulk_out_endpointAddress);

	ret = ch348_detect_version(serial);
	if (ret)
		goto err_free_ch348;

	ret = ch348_submit_urbs(serial);
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

	cancel_work_sync(&ch348->write_work);
	ch348_kill_urbs(serial);

	kfree(ch348);
}

static int ch348_calc_num_ports(struct usb_serial *serial,
				struct usb_serial_endpoints *epds)
{
	int i;

	epds->num_bulk_out = CH348_MAXPORT;

	for (i = serial->type->num_bulk_out; i < CH348_MAXPORT; ++i)
		epds->bulk_out[i] = epds->bulk_out[0];

	return CH348_MAXPORT;
}

static int ch348_suspend(struct usb_serial *serial, pm_message_t message)
{
	struct ch348 *ch348 = usb_get_serial_data(serial);

	cancel_work_sync(&ch348->write_work);

	return 0;
}

static int ch348_resume(struct usb_serial *serial)
{
	struct ch348 *ch348 = usb_get_serial_data(serial);
	int ret;

	ret = ch348_submit_urbs(serial);
	if (ret)
		return ret;

	schedule_work(&ch348->write_work);

	return 0;
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
	.num_bulk_in =		2,
	.num_bulk_out =		2,
	.open =			ch348_open,
	.close =		ch348_close,
	.set_termios =		ch348_set_termios,
	.process_read_urb =	ch348_process_read_urb,
	.write =		ch348_write,
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
