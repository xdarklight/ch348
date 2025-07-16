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

#include <linux/bitops.h>
#include <linux/cleanup.h>
#include <linux/completion.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/gpio/driver.h>
#include <linux/kernel.h>
#include <linux/kfifo.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/refcount.h>
#include <linux/serial.h>
#include <linux/serial_reg.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/usb.h>
#include <linux/usb/serial.h>
#include <linux/workqueue.h>

#define CH348_CMD_TIMEOUT	5000

#define CH348_CTO_D	0x01
#define CH348_CTO_R	0x02

#define CH348_CTI_C	0x10
#define CH348_CTI_DSR	0x20
#define CH348_CTI_R	0x40
#define CH348_CTI_DCD	0x80

#define CMD_W_R		0xc0
#define CMD_W_BR	0x80

#define CMD_WB_E	0x90
#define CMD_RB_E	0xc0

/* R_C1 = 0x01 is UART_IER compatible */

#define R_C2		0x02
#define R_C2_ACTIVATE	0x87 /* no official documentation available */

#define R_C3		0x03

#define R_C4		0x04
#define R_C4_UNKNOWN00	0x00 /* no official documentation available */
#define R_C4_UNKNOWN01	0x01 /* no official documentation available */
#define R_C4_UNKNOWN10	0x10 /* no official documentation available */
#define R_C4_UNKNOWN11	0x11 /* no official documentation available */
#define R_C4_ACTIVATE	0x08 /* no official documentation available */
#define R_C4_HW_FLOW	0x50
#define R_C4_NO_RTS	0x51 /* no official documentation, name is a guess */

#define R_C5		0x06
#define R_MOD		0x97
#define R_IO_D		0x98
#define R_IO_O		0x99
#define R_IO_I		0x9b
#define R_TM_O		0x9c
#define R_INIT		0xa1
#define R_IO_CE		0xa3
#define R_IO_CD		0xa4
#define R_IO_CO		0xa5
#define R_IO_CI		0xa7
#define R_IO_RE		0xaa
#define R_IO_RD		0xab

#define CMD_VER		0x96

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

enum ch348_package {
	CH348Q, /* LQFP48 (small) */
	CH348L, /* LQFP100 (large) */
};

/**
 * struct ch348_port - per-port information
 * @baudrate:		A cached copy of current baudrate for the RX logic
 * @port:		Pointer to the struct usb_serial_port
 * @tx_timeout:		Timer when the TX will time out
 * @hw_flow_control:	Whether HW flow control is enabled or disabled
 */
struct ch348_port {
	speed_t baudrate;
	struct usb_serial_port *port;
	struct timer_list tx_timeout;
	bool hw_flow_control;
};

#define CH348_NUM_GPIO	48

/**
 * struct ch348 - main container for all this driver information
 * @serial:		pointer to the serial structure
 * @ports:		List of per-port information
 * @write_work:		worker for processing the write queues
 * @tx_ep:		endpoint number for serial data transmit/write operation
 * @config_ep:		endpoint number for configure operations
 * @package_type:	indicates package type
 * @gc:			gpio chip for GPIO access
 * gpiochip_registered:	indicates that the gpio chip was successfully registered
 * @gpio_value_lock:	avoids concurrent gpio_{enabled,direction,out_value} access
 * @gpio_enabled_mask:	bitmap of pins are in GPIO mode
 * @gpio_dir_mask:	bitmap of pins in output (1) or input mode (0)
 * @gpio_out_mask:	bitmap of pins and their corresponding GPIO output value
 * @gpio_in_lock:	protects against concurrent hardware GPIO reads
 * @gpio_in_completion:	indicates that the GPIO input values have been read
 * @gpio_in_mask:	bitmap of GPIOs and their input values
 */
struct ch348 {
	struct usb_serial *serial;
	struct ch348_port ports[CH348_MAXPORT];

	struct work_struct write_work;

	int tx_ep;
	int config_ep;

	enum ch348_package package_type;

	struct gpio_chip gc;
	const char *gpio_names[CH348_NUM_GPIO];
	bool gpiochip_registered;

	struct mutex gpio_value_lock;
	DECLARE_BITMAP(gpio_enabled_mask, CH348_NUM_GPIO);
	DECLARE_BITMAP(gpio_dir_mask, CH348_NUM_GPIO);
	DECLARE_BITMAP(gpio_out_mask, CH348_NUM_GPIO);

	struct mutex gpio_in_lock;
	struct completion gpio_in_completion;
	DECLARE_BITMAP(gpio_in_mask, CH348_NUM_GPIO);
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
		u8 unknown;
		u8 lsr_signal;
		u8 modem_signal;
		__le64 gpio_in_mask;
		struct ch348_config_data_init init_data;
	} data;
} __packed;

#define CH348_STATUS_ENTRY_PORTNUM_MASK		0xf

enum ch348_gpio_direction {
	CH348_GPIO_DIRECTION_CONFIGURABLE,
	CH348_GPIO_DIRECTION_INPUT_ONLY,
	CH348_GPIO_DIRECTION_OUTPUT_ONLY,
};

struct ch348_gpio_data {
	u8 port_number;
	enum ch348_gpio_direction direction;
	const char *name;
};

#define CH348_GPIO_DATA(_gpio, _name, _port_number, _direction)	\
	[_gpio] = {						\
		.port_number = _port_number,			\
		.direction = CH348_GPIO_DIRECTION_##_direction,	\
		.name = _name #_port_number " / GPIO" #_gpio,	\
	}

static const struct ch348_gpio_data ch348_gpios[CH348_NUM_GPIO] = {
	CH348_GPIO_DATA(0, "CTS", 0, INPUT_ONLY),
	CH348_GPIO_DATA(1, "RTS", 0, OUTPUT_ONLY),
	CH348_GPIO_DATA(2, "CTS", 1, INPUT_ONLY),
	CH348_GPIO_DATA(3, "RTS", 1, OUTPUT_ONLY),
	CH348_GPIO_DATA(4, "CTS", 2, INPUT_ONLY),
	CH348_GPIO_DATA(5, "RTS", 2, OUTPUT_ONLY),
	CH348_GPIO_DATA(6, "CTS", 3, INPUT_ONLY),
	CH348_GPIO_DATA(7, "RTS", 3, OUTPUT_ONLY),
	CH348_GPIO_DATA(8, "DTR", 0, CONFIGURABLE),
	CH348_GPIO_DATA(9, "DTR", 1, CONFIGURABLE),
	CH348_GPIO_DATA(10, "DTR", 2, CONFIGURABLE),
	CH348_GPIO_DATA(11, "DTR", 3, CONFIGURABLE),
	CH348_GPIO_DATA(12, "CTS", 4, INPUT_ONLY),
	CH348_GPIO_DATA(13, "RTS", 4, OUTPUT_ONLY),
	CH348_GPIO_DATA(14, "CTS", 5, INPUT_ONLY),
	CH348_GPIO_DATA(15, "RTS", 5, OUTPUT_ONLY),
	CH348_GPIO_DATA(16, "CTS", 6, INPUT_ONLY),
	CH348_GPIO_DATA(17, "RTS", 6, OUTPUT_ONLY),
	CH348_GPIO_DATA(18, "CTS", 7, INPUT_ONLY),
	CH348_GPIO_DATA(19, "RTS", 7, OUTPUT_ONLY),
	CH348_GPIO_DATA(20, "DTR", 4, CONFIGURABLE),
	CH348_GPIO_DATA(21, "DTR", 5, CONFIGURABLE),
	CH348_GPIO_DATA(22, "DTR", 6, CONFIGURABLE),
	CH348_GPIO_DATA(23, "DTR", 7, CONFIGURABLE),
	CH348_GPIO_DATA(24, "DSR", 0, INPUT_ONLY),
	CH348_GPIO_DATA(25, "RI", 0, INPUT_ONLY),
	CH348_GPIO_DATA(26, "DCD", 0, INPUT_ONLY),
	CH348_GPIO_DATA(27, "DSR", 1, INPUT_ONLY),
	CH348_GPIO_DATA(28, "RI", 1, INPUT_ONLY),
	CH348_GPIO_DATA(29, "DCD", 1, INPUT_ONLY),
	CH348_GPIO_DATA(30, "DSR", 2, INPUT_ONLY),
	CH348_GPIO_DATA(31, "DCD", 2, INPUT_ONLY),
	CH348_GPIO_DATA(32, "RI", 2, INPUT_ONLY),
	CH348_GPIO_DATA(33, "DSR", 3, INPUT_ONLY),
	CH348_GPIO_DATA(34, "DCD", 3, INPUT_ONLY),
	CH348_GPIO_DATA(35, "RI", 3, INPUT_ONLY),
	CH348_GPIO_DATA(36, "DSR", 4, INPUT_ONLY),
	CH348_GPIO_DATA(37, "DCD", 4, INPUT_ONLY),
	CH348_GPIO_DATA(38, "RI", 4, INPUT_ONLY),
	CH348_GPIO_DATA(39, "DSR", 5, INPUT_ONLY),
	CH348_GPIO_DATA(40, "DCD", 5, INPUT_ONLY),
	CH348_GPIO_DATA(41, "RI", 5, INPUT_ONLY),
	CH348_GPIO_DATA(42, "DSR", 6, INPUT_ONLY),
	CH348_GPIO_DATA(43, "DCD", 6, INPUT_ONLY),
	CH348_GPIO_DATA(44, "RI", 6, INPUT_ONLY),
	CH348_GPIO_DATA(45, "DSR", 7, INPUT_ONLY),
	CH348_GPIO_DATA(46, "DCD", 7, INPUT_ONLY),
	CH348_GPIO_DATA(47, "RI", 7, INPUT_ONLY),
};

static int ch348_submit_urbs(struct usb_serial *serial)
{
	int ret;

	ret = usb_serial_generic_open(NULL,
				      serial->port[CH348_PORTNUM_SERIAL_RX_TX]);
	if (ret) {
		dev_err(&serial->dev->dev, "Failed to open RX/TX port: %d\n",
			ret);
		return ret;
	}

	ret = usb_serial_generic_open(NULL,
				      serial->port[CH348_PORTNUM_STATUS_INT_CONFIG]);
	if (ret) {
		dev_err(&serial->dev->dev,
			"Failed to submit STATUS/INT URB: %d\n", ret);
		usb_serial_generic_close(serial->port[CH348_PORTNUM_SERIAL_RX_TX]);
		return ret;
	}

	return 0;
}

static void ch348_kill_urbs(struct usb_serial *serial)
{
	usb_serial_generic_close(serial->port[CH348_PORTNUM_STATUS_INT_CONFIG]);
	usb_serial_generic_close(serial->port[CH348_PORTNUM_SERIAL_RX_TX]);
}

static void ch348_write_done(struct usb_serial_port *port)
{
	struct ch348_port *ch348_p = usb_get_serial_port_data(port);
	struct ch348 *ch348 = usb_get_serial_data(port->serial);

	timer_delete(&ch348_p->tx_timeout);

	usb_serial_port_softint(port);

	if (!kfifo_is_empty(&port->write_fifo))
		schedule_work(&ch348->write_work);
}

static void ch348_tx_timeout(struct timer_list *t)
{
	struct ch348_port *ch348_p = from_timer(ch348_p, t, tx_timeout); /* TODO: needs to be timer_container_of() for Linux 6.16 */

	dev_err_console(ch348_p->port, "Writing TX buffer timed out\n");

	ch348_write_done(ch348_p->port);
}

static void ch348_write_work(struct work_struct *work)
{
	struct ch348 *ch348 = container_of(work, struct ch348, write_work);
	struct usb_serial_port *port, *hw_tx_port;
	struct ch348_port *ch348_p;
	struct ch348_txbuf *rxt;
	unsigned int i, count;
	unsigned long flags;
	int ret;

	hw_tx_port = ch348->serial->port[CH348_PORTNUM_SERIAL_RX_TX];
	rxt = hw_tx_port->write_urbs[0]->transfer_buffer;

	for (i = 0; i < CH348_MAXPORT; i++) {
		port = ch348->serial->port[i];
		ch348_p = usb_get_serial_port_data(port);

		if (timer_pending(&ch348_p->tx_timeout)) {
			/* Previous TX is still pending */
			continue;
		}

		/*
		 * Only ingest as many bytes as we can transfer with
		 * one URB at a time keeping the TX header in mind.
		 */
		count = kfifo_out_locked(&port->write_fifo, rxt->data,
					 hw_tx_port->bulk_out_size - CH348_TX_HDRSIZE,
					 &port->lock);
		if (!count)
			continue;

		rxt->port = port->port_number;
		rxt->length = cpu_to_le16(count);

		spin_lock_irqsave(&port->lock, flags);
		port->tx_bytes += count;
		spin_unlock_irqrestore(&port->lock, flags);

		usb_serial_debug_data(&port->dev, __func__,
				      count + CH348_TX_HDRSIZE,
				      (const unsigned char *)rxt);

		mod_timer(&ch348_p->tx_timeout,
			  jiffies + msecs_to_jiffies(CH348_CMD_TIMEOUT * 2));

		ret = usb_bulk_msg(ch348->serial->dev, ch348->tx_ep, rxt,
				   count + CH348_TX_HDRSIZE, NULL,
				   CH348_CMD_TIMEOUT);
		if (ret) {
			dev_err_console(port,
					"Failed to bulk write TX buffer: %d\n",
					ret);
			ch348_write_done(port);
		}

		spin_lock_irqsave(&port->lock, flags);
		port->tx_bytes -= count;
		spin_unlock_irqrestore(&port->lock, flags);

		port->icount.tx += count;
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
			status_len += sizeof(status_entry->data.unknown);
			dev_dbg(&port->dev, "Ignoring status with zero reg_iir\n");
		} else if (status_entry->reg_iir == R_INIT) {
			status_len += sizeof(status_entry->data.init_data);
		} else if (status_entry->reg_iir == R_IO_CI) {
			u64 val;

			status_len += sizeof(status_entry->data.gpio_in_mask);

			val = le64_to_cpu(status_entry->data.gpio_in_mask);
			bitmap_from_arr64(ch348->gpio_in_mask, &val,
					  ch348->gc.ngpio);

			complete_all(&ch348->gpio_in_completion);
		} else if (status_entry->reg_iir == R_IO_CD ||
			   status_entry->reg_iir == R_IO_CO) {
			/* nothing to do - just skip this entry */
			status_len += sizeof(status_entry->data.unknown);
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
			ch348_write_done(port);
		} else {
			status_len += sizeof(status_entry->data.unknown);
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

	if (port->port_number == CH348_PORTNUM_SERIAL_RX_TX)
		ch348_process_serial_rx_urb(port->serial, urb);
	else if (port->port_number == CH348_PORTNUM_STATUS_INT_CONFIG)
		ch348_process_status_urb(port->serial, urb);
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

static int ch348_write_config_be64(struct ch348 *ch348, u8 cmd, u8 reg, u64 val)
{
	__be64 config_val = cpu_to_be64(val);

	return ch348_write_config(ch348, cmd, reg, &config_val,
				  sizeof(config_val));
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

static void ch348_set_flow_control(struct usb_serial_port *port,
				   struct ktermios *termios,
				   const struct ktermios *termios_old)
{
	struct ch348_port *ch348_p = usb_get_serial_port_data(port);
	struct ch348 *ch348 = usb_get_serial_data(port->serial);
	bool hw_flow_control = !!(termios->c_cflag & CRTSCTS);
	int ret;

	if (ch348_p->hw_flow_control == hw_flow_control)
		return;

	if (hw_flow_control && ch348->package_type == CH348Q &&
	    port->port_number >= 4) {
		dev_err(&ch348->serial->dev->dev,
			"Flow control is not supported on CH348Q port %u\n",
			port->port_number);
		termios->c_cflag &= ~CRTSCTS;
		return;
	}

	ret = ch348_port_config(port, CMD_W_BR, R_C4,
				hw_flow_control ? R_C4_HW_FLOW : R_C4_NO_RTS);
	if (ret) {
		if (termios_old) {
			termios->c_cflag &= ~CRTSCTS;
			termios->c_cflag |= (termios_old->c_cflag & CRTSCTS);
		}

		return;
	}

	ch348_p->hw_flow_control = hw_flow_control;
}

static void ch348_set_termios(struct tty_struct *tty, struct usb_serial_port *port,
			      const struct ktermios *termios_old)
{
	struct ch348_port *ch348_p = usb_get_serial_port_data(port);
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
	ch348_p->baudrate = baudrate;

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
		dev_err(&ch348->serial->dev->dev,
			"Failed to change line settings: %d\n", ret);
		if (termios_old)
			tty->termios = *termios_old;
	}

	ch348_port_config(port, CMD_W_R, UART_IER, UART_IER_RDI |
			  UART_IER_THRI | UART_IER_RLSI | UART_IER_MSI);

	ch348_set_flow_control(port, termios, termios_old);
}

static void ch348_dtr_rts(struct usb_serial_port *port, int on)
{
	struct ch348 *ch348 = usb_get_serial_data(port->serial);
	int ret;

	/*
	 * Only the first four ports have the modem control pins routed outside
	 * the package.
	 */
	if (ch348->package_type == CH348Q && port->port_number >= 4) {
		dev_dbg(&port->serial->dev->dev,
			"DTR/RTS is not supported on CH348 port %u\n",
			port->port_number);
		return;
	}

	ret = ch348_port_config(port, CMD_W_BR, R_C4, R_C4_UNKNOWN01);
	if (ret)
		dev_err(&port->serial->dev->dev,
			"Failed to configure R_C4_UNKNOWN01: %d\n", ret);

	ret = ch348_port_config(port, CMD_W_BR, R_C4, R_C4_UNKNOWN11);
	if (ret)
		dev_err(&port->serial->dev->dev,
			"Failed to configure R_C4_UNKNOWN11: %d\n", ret);
}

static int ch348_open(struct tty_struct *tty, struct usb_serial_port *port)
{
	int ret;

	if (tty)
		ch348_set_termios(tty, port, NULL);

	ret = ch348_port_config(port, CMD_W_R, R_C2, R_C2_ACTIVATE);
	if (ret) {
		dev_err(&port->serial->dev->dev,
			"Failed to configure R_C2_ACTIVATE: %d\n", ret);
		return ret;
	}

	ret = ch348_port_config(port, CMD_W_R, R_C4, R_C4_ACTIVATE);
	if (ret) {
		dev_err(&port->serial->dev->dev,
			"Failed to configure R_C4_ACTIVATE: %d\n", ret);
		return ret;
	}

	return 0;
}

static void ch348_close(struct usb_serial_port *port)
{
	struct ch348_port *ch348_p = usb_get_serial_port_data(port);
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
	kfifo_reset_out(&port->write_fifo);
	spin_unlock_irqrestore(&port->lock, flags);

	timer_shutdown_sync(&ch348_p->tx_timeout);
}

static int ch348_port_probe(struct usb_serial_port *port)
{
	struct ch348 *ch348 = usb_get_serial_data(port->serial);
	struct ch348_port *ch348_p = &ch348->ports[port->port_number];

	ch348_p->port = port;
	timer_setup(&ch348_p->tx_timeout, ch348_tx_timeout, 0);

	usb_set_serial_port_data(port, ch348_p);

	return 0;
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

	ch348->package_type = (version_buf[1] & 0x80) ? CH348Q : CH348L;

	dev_info(&serial->dev->dev, "Found WCH CH348%c version 0x%02x\n",
		 ch348->package_type == CH348Q ? 'Q' : 'L', version_buf[0]);

out:
	kfree(version_buf);

	return ret;
}

static int ch348_gpio_enable(struct gpio_chip *gc, unsigned int offset,
			     bool enable)
{
	struct ch348 *ch348 = gpiochip_get_data(gc);
	u64 val;

	scoped_guard(mutex, &ch348->gpio_value_lock) {
		if (enable)
			set_bit(offset, ch348->gpio_enabled_mask);
		else
			clear_bit(offset, ch348->gpio_enabled_mask);

		bitmap_to_arr64(&val, ch348->gpio_enabled_mask, gc->ngpio);

		return ch348_write_config_be64(ch348, R_MOD, R_IO_CE, val);
	}
}

static int ch348_gpio_request(struct gpio_chip *gc, unsigned int offset)
{
	return ch348_gpio_enable(gc, offset, true);
}

static void ch348_gpio_free(struct gpio_chip *gc, unsigned int offset)
{
	ch348_gpio_enable(gc, offset, false);
}

static int ch348_gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	struct ch348 *ch348 = gpiochip_get_data(gc);
	bool is_output;

	if (ch348_gpios[offset].direction == CH348_GPIO_DIRECTION_INPUT_ONLY)
		return GPIO_LINE_DIRECTION_IN;

	if (ch348_gpios[offset].direction == CH348_GPIO_DIRECTION_OUTPUT_ONLY)
		return GPIO_LINE_DIRECTION_OUT;

	scoped_guard(mutex, &ch348->gpio_value_lock)
		is_output = test_bit(offset, ch348->gpio_dir_mask);

	return is_output ? GPIO_LINE_DIRECTION_OUT : GPIO_LINE_DIRECTION_IN;
}

static int ch348_gpio_set_direction(struct gpio_chip *gc, unsigned int offset,
				    unsigned int direction)
{
	struct ch348 *ch348 = gpiochip_get_data(gc);
	u64 val;

	scoped_guard(mutex, &ch348->gpio_value_lock) {
		if (direction == GPIO_LINE_DIRECTION_OUT)
			set_bit(offset, ch348->gpio_dir_mask);
		else
			clear_bit(offset, ch348->gpio_dir_mask);

		bitmap_to_arr64(&val, ch348->gpio_dir_mask, gc->ngpio);

		return ch348_write_config_be64(ch348, R_MOD, R_IO_CD, val);
	}
}

static int ch348_gpio_direction_input(struct gpio_chip *gc, unsigned int offset)
{
	if (ch348_gpios[offset].direction == CH348_GPIO_DIRECTION_OUTPUT_ONLY)
		return -EINVAL;

	return ch348_gpio_set_direction(gc, offset, GPIO_LINE_DIRECTION_IN);
}

static int ch348_gpio_direction_output(struct gpio_chip *gc,
				       unsigned int offset, int value)
{
	struct ch348 *ch348 = gpiochip_get_data(gc);
	int ret;

	if (ch348_gpios[offset].direction == CH348_GPIO_DIRECTION_INPUT_ONLY)
		return -EINVAL;

	ret = ch348_gpio_set_direction(gc, offset, GPIO_LINE_DIRECTION_OUT);
	if (ret)
		return ret;

	return ch348->gc.set_rv(gc, offset, value);
}

static int ch348_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	DECLARE_BITMAP(mask, CH348_NUM_GPIO) = { 0 };
	DECLARE_BITMAP(bits, CH348_NUM_GPIO) = { 0 };
	struct ch348 *ch348 = gpiochip_get_data(gc);
	int ret;

	set_bit(offset, mask);

	ret = ch348->gc.get_multiple(gc, mask, bits);
	if (ret)
		return ret;

	return test_bit(offset, bits);
}

static int ch348_gpio_set(struct gpio_chip *gc, unsigned int offset, int value)
{
	DECLARE_BITMAP(mask, CH348_NUM_GPIO) = { 0 };
	DECLARE_BITMAP(bits, CH348_NUM_GPIO) = { 0 };
	struct ch348 *ch348 = gpiochip_get_data(gc);

	set_bit(offset, mask);

	if (value)
		set_bit(offset, bits);

	return ch348->gc.set_multiple_rv(gc, mask, bits);
}

static int ch348_gpio_get_multiple(struct gpio_chip *gc, unsigned long *mask,
				   unsigned long *bits)
{
	unsigned long jiffies = msecs_to_jiffies(CH348_CMD_TIMEOUT);
	struct ch348 *ch348 = gpiochip_get_data(gc);
	int ret;

	scoped_guard(mutex, &ch348->gpio_in_lock) {
		reinit_completion(&ch348->gpio_in_completion);

		ret = ch348_write_config(ch348, R_MOD, R_IO_CI, NULL, 0);
		if (ret)
			return ret;

		if (!wait_for_completion_timeout(&ch348->gpio_in_completion,
						 jiffies))
			return -ETIMEDOUT;

		bitmap_and(bits, ch348->gpio_in_mask, mask, gc->ngpio);
	}

	return 0;
}

static int ch348_gpio_set_multiple(struct gpio_chip *gc, unsigned long *mask,
				   unsigned long *bits)
{
	struct ch348 *ch348 = gpiochip_get_data(gc);
	u64 val;

	scoped_guard(mutex, &ch348->gpio_value_lock) {
		bitmap_replace(ch348->gpio_out_mask, ch348->gpio_out_mask,
			       bits, mask, gc->ngpio);

		bitmap_to_arr64(&val, ch348->gpio_out_mask, gc->ngpio);

		return ch348_write_config_be64(ch348, R_MOD, R_IO_CO, val);
	}
}

static int ch348_attach(struct usb_serial *serial)
{
	struct usb_serial_port *tx_port, *config_port;
	struct ch348 *ch348;
	unsigned int i;
	int ret;

	ch348 = kzalloc(sizeof(*ch348), GFP_KERNEL);
	if (!ch348)
		return -ENOMEM;

	usb_set_serial_data(serial, ch348);

	ch348->serial = serial;

	INIT_WORK(&ch348->write_work, ch348_write_work);

	init_completion(&ch348->gpio_in_completion);

	devm_mutex_init(&serial->dev->dev, &ch348->gpio_value_lock);
	devm_mutex_init(&serial->dev->dev, &ch348->gpio_in_lock);

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

	for (i = 0; i < ARRAY_SIZE(ch348->gpio_names); i++)
		ch348->gpio_names[i] = ch348_gpios[i].name;

	ch348->gc.request = ch348_gpio_request;
	ch348->gc.free = ch348_gpio_free;
	ch348->gc.get_direction = ch348_gpio_get_direction;
	ch348->gc.direction_input = ch348_gpio_direction_input;
	ch348->gc.direction_output = ch348_gpio_direction_output;
	ch348->gc.get = ch348_gpio_get;
	ch348->gc.set_rv = ch348_gpio_set;
	ch348->gc.get_multiple = ch348_gpio_get_multiple;
	ch348->gc.set_multiple_rv = ch348_gpio_set_multiple;
	ch348->gc.owner = THIS_MODULE;
	ch348->gc.parent = &serial->dev->dev;
	ch348->gc.base = -1;
	ch348->gc.can_sleep = true;
	ch348->gc.names = ch348->gpio_names;

	ch348->gc.ngpio = ch348->package_type == CH348Q ? 12 : CH348_NUM_GPIO;
	ch348->gc.label = ch348->package_type == CH348Q ? "CH348Q" : "CH348L";

	ret = gpiochip_add_data(&ch348->gc, ch348);
	if (ret)
		dev_info(&serial->dev->dev,
			 "GPIO controller registration failed: %d\n", ret);
	else
		ch348->gpiochip_registered = true;

	return 0;

err_free_ch348:
	kfree(ch348);
	return ret;
}

static void ch348_release(struct usb_serial *serial)
{
	struct ch348 *ch348 = usb_get_serial_data(serial);

	if (ch348->gpiochip_registered)
		gpiochip_remove(&ch348->gc);

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

	schedule_work(&ch348->write_work);

	return 0;
}

static const struct usb_device_id ch348_ids[] = {
	{ USB_DEVICE(0x1a86, 0x55d9) },
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
	.dtr_rts =		ch348_dtr_rts,
	.process_read_urb =	ch348_process_read_urb,
	.write =		ch348_write,
	.calc_num_ports =	ch348_calc_num_ports,
	.port_probe =		ch348_port_probe,
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
