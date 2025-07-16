// SPDX-License-Identifier: GPL-2.0
/*
 * USB serial driver for USB to Octal UARTs chip ch348.
 *
 * Copyright (C) 2023 Corentin Labbe <clabbe@baylibre.com>
 * With the help of Neil Armstrong <neil.armstrong@linaro.org>
 * Copyright (C) 2026 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * Based on the ch9344 driver:
 *   https://github.com/WCHSoftGroup/ch9344ser_linux/
 *   Copyright (C) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
 */

#include <linux/bitmap.h>
#include <linux/bitops.h>
#include <linux/cleanup.h>
#include <linux/completion.h>
#include <linux/errno.h>
#include <linux/gpio/driver.h>
#include <linux/kernel.h>
#include <linux/kfifo.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/serial.h>
#include <linux/serial_reg.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string_choices.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/usb.h>
#include <linux/usb/serial.h>

#define CH348_CMD_TIMEOUT		2000

#define CMD_W_R				0xc0
#define CMD_W_BR			0x80

#define CMD_WB_E			0x90

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
#define R_C4_HW_FLOW_CONTROL_OFF	0x50
#define R_C4_HW_FLOW_CONTROL_ON		0x51


#define R_C5				0x06

#define VEN_R				0x85
#define VEN_R_UPDATE_MODEM_STATUS	0x06

#define VEN_W				0x8a

#define CMD_VER				0x96

#define R_MOD				0x97
#define R_IO_D				0x98
#define R_IO_O				0x99
#define R_IO_I				0x9b
#define R_TM_O				0x9c
#define R_INIT				0xa1
#define R_IO_CE				0xa3
#define R_IO_CD				0xa4
#define R_IO_CO				0xa5
#define R_IO_CI				0xa7
#define R_IO_RE				0xaa
#define R_IO_RD				0xab

/*
 * The CH348 multiplexes rx & tx into a pair of Bulk USB endpoints for the 8
 * serial ports, and another pair of Bulk USB endpoints to set port settings
 * and receive port status events.
 *
 * The USB serial cores ties every Bulk endpoints pairs to each ports, In our
 * case it will set port 0 with the rx/tx endpoints. Port 1 is configured with
 * the status/int endpoint while we move config (write) to a virtual port
 * (which is not visible to userspace) to simplify the TX handling (because we
 * can then let USB serial core set up a write URB for each of the actual ports
 * which then does serial TX).
 * For serial TX we implement our own .write callback because we need to not
 * only wait for the URB to complete but also for the UART_IIR_THRI signal.
 *
 * For bulk reads we use USB serial core's helpers, even for the status/int
 * handling as it simplifies our code.
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

#define CH348_NUM_GPIO	48

/**
 * struct ch348 - main container for all this driver information
 * @open_ports:		bitmap of ports that are currently opened
 * @open_ports_lock:	protect against concurrent modification of open_ports
 * @package_type:	indicates package type
 * @gc:			gpio chip for GPIO access
 * @gpiochip_registered:	indicates that the gpio chip was successfully
 * 				registered
 * @gpio_value_lock:	avoids concurrent gpio_{enabled,direction,out_value} access
 * @gpio_enabled_mask:	bitmap of pins are in GPIO mode
 * @gpio_dir_mask:	bitmap of pins in output (1) or input mode (0)
 * @gpio_out_mask:	bitmap of pins and their corresponding GPIO output value
 * @gpio_in_lock:	protects against concurrent hardware GPIO reads
 * @gpio_in_completion:	indicates that the GPIO input values have been read
 * @gpio_in_mask:	bitmap of GPIOs and their input values
 */
struct ch348 {
	DECLARE_BITMAP(open_ports, CH348_MAXPORT);
	struct mutex open_ports_lock;

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
	u8 recv_tmt;
	u8 unknown;
} __packed;

#define CH348_CONFIG_DATA_INIT_FORMAT_ONE_STOPBIT	0x0
#define CH348_CONFIG_DATA_INIT_FORMAT_TWO_STOPBITS	0x2

struct ch348_ven_r_msr {
	u8 control;
	u8 msr;
} __packed;

struct ch348_status_entry {
	u8 portnum;
	u8 reg_iir;
	union {
		u8 unknown;
		u8 lsr;
		u8 msr;
		struct ch348_ven_r_msr ven_r_msr;
		__le64 gpio_in_mask;
		struct ch348_config_data_init init_data;
	} data;
} __packed;

#define CH348_STATUS_HDRSIZE	offsetof(struct ch348_status_entry, data)
#define CH348_STATUS_ENTRY_PORTNUM_MASK			0xf

enum ch348_status_action {
	CH348_STATUS_ACTION_NONE,
	CH348_STATUS_ACTION_UART_IIR_RLSI,
	CH348_STATUS_ACTION_UART_IIR_THRI,
	CH348_STATUS_ACTION_UPDATE_GPIO_IN,
};

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

static void ch348_kill_port_read_urbs(struct usb_serial_port *port)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(port->read_urbs); ++i)
		usb_kill_urb(port->read_urbs[i]);
}

static int ch348_submit_read_urbs(struct usb_serial *serial, gfp_t mem_flags)
{
	struct usb_serial_port *rx_port, *status_int_port;
	int ret;

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

static void ch348_clear_write_state(struct usb_serial_port *port)
{
	scoped_guard(spinlock_irqsave, &port->lock) {
		port->tx_bytes = 0;
		clear_bit_unlock(USB_SERIAL_WRITE_BUSY, &port->flags);
	}
}

static int ch348_write_start(struct usb_serial_port *port, gfp_t mem_flags)
{
	struct ch348_txbuf *txb;
	unsigned int tx_bytes;
	int ret;

	if (test_and_set_bit_lock(USB_SERIAL_WRITE_BUSY, &port->flags))
		return 0;

	txb = port->write_urb->transfer_buffer;

	scoped_guard(spinlock_irqsave, &port->lock) {
		/*
		 * Only ingest as many bytes as we can transfer with one URB at
		 * a time keeping the TX header in mind.
		 */
		tx_bytes = kfifo_out(&port->write_fifo, txb->data,
				     port->bulk_out_size - CH348_TX_HDRSIZE);
		if (!tx_bytes) {
			clear_bit_unlock(USB_SERIAL_WRITE_BUSY, &port->flags);
			return 0;
		}

		port->tx_bytes += tx_bytes;
	}

	port->write_urb->transfer_buffer_length = tx_bytes + CH348_TX_HDRSIZE;

	txb->port = port->port_number;
	txb->length = cpu_to_le16(tx_bytes);

	usb_serial_debug_data(&port->dev, __func__,
			      port->write_urb->transfer_buffer_length,
			      port->write_urb->transfer_buffer);

	ret = usb_submit_urb(port->write_urb, mem_flags);
	if (ret) {
		dev_err_console(port, "Failed to submit write URB: %d\n", ret);
		ch348_clear_write_state(port);
	}

	return ret;
}

static void ch348_write_done(struct usb_serial_port *port)
{
	scoped_guard(spinlock_irqsave, &port->lock)
		port->icount.tx += port->tx_bytes;

	ch348_clear_write_state(port);

	ch348_write_start(port, GFP_ATOMIC);
	usb_serial_port_softint(port);
}

static void ch348_process_status_urb(struct usb_serial *serial, struct urb *urb)
{
	struct ch348_status_entry *status_entry;
	enum ch348_status_action action;
	struct usb_serial_port *port;
	unsigned int i;
	u8 portnum;

	if (urb->actual_length <= CH348_STATUS_HDRSIZE) {
		dev_dbg_ratelimited(&serial->dev->dev,
				    "Received too short status buffer with %u bytes\n",
				    urb->actual_length);
		return;
	}

	for (i = 0; (i + CH348_STATUS_HDRSIZE) <= urb->actual_length;) {
		status_entry = urb->transfer_buffer + i;
		portnum = status_entry->portnum & CH348_STATUS_ENTRY_PORTNUM_MASK;

		if (portnum >= CH348_MAXPORT) {
			dev_dbg_ratelimited(&serial->dev->dev,
					    "Invalid port %d in status entry\n",
					    portnum);
			break;
		}

		action = CH348_STATUS_ACTION_NONE;
		port = serial->port[portnum];
		i += CH348_STATUS_HDRSIZE;

		if (status_entry->reg_iir == R_INIT) {
			i += sizeof(status_entry->data.init_data);
		} else if (status_entry->reg_iir == VEN_R) {
			i += sizeof(status_entry->data.ven_r_msr);
		} else if (status_entry->reg_iir == R_IO_CI) {
			i += sizeof(status_entry->data.gpio_in_mask);
			action = CH348_STATUS_ACTION_UPDATE_GPIO_IN;
		} else if (status_entry->reg_iir == R_IO_CD ||
			   status_entry->reg_iir == R_IO_CO) {
			i += sizeof(status_entry->data.unknown);
		} else if ((status_entry->reg_iir & UART_IIR_ID) == UART_IIR_RLSI) {
			i += sizeof(status_entry->data.lsr);
			action = CH348_STATUS_ACTION_UART_IIR_RLSI;
		} else if ((status_entry->reg_iir & UART_IIR_ID) == UART_IIR_THRI) {
			i += sizeof(status_entry->data.unknown);
			action = CH348_STATUS_ACTION_UART_IIR_THRI;
		} else if ((status_entry->reg_iir & UART_IIR_ID) == UART_IIR_MSI) {
			i += sizeof(status_entry->data.msr);
		} else {
			i += sizeof(status_entry->data.unknown);
			dev_dbg_ratelimited(&port->dev,
					    "Unsupported status with reg_iir 0x%02x\n",
					    status_entry->reg_iir);
		}

		if (urb->actual_length < i) {
			dev_dbg_ratelimited(&port->dev,
					    "Truncated status with reg_iir 0x%02x\n",
					    status_entry->reg_iir);
			break;
		}

		if (action == CH348_STATUS_ACTION_UART_IIR_RLSI) {
			if (status_entry->data.lsr & UART_LSR_OE)
				port->icount.overrun++;
			if (status_entry->data.lsr & UART_LSR_PE)
				port->icount.parity++;
			if (status_entry->data.lsr & UART_LSR_FE)
				port->icount.frame++;
			if (status_entry->data.lsr & UART_LSR_BI)
				port->icount.brk++;
		} else if (action == CH348_STATUS_ACTION_UART_IIR_THRI) {
			ch348_write_done(port);
		} else if (action == CH348_STATUS_ACTION_UPDATE_GPIO_IN) {
			u64 val = le64_to_cpu(status_entry->data.gpio_in_mask);
			struct ch348 *ch348 = usb_get_serial_data(serial);

			bitmap_from_arr64(ch348->gpio_in_mask, &val,
					  ch348->gc.ngpio);

			complete_all(&ch348->gpio_in_completion);
		}
	}
}

static void ch348_process_serial_rx_urb(struct usb_serial *serial,
					struct urb *urb)
{
	unsigned int portnum, serial_rx_len, i;
	struct usb_serial_port *port;
	struct ch348_rxbuf *rxb;

	if (urb->actual_length < sizeof(*rxb)) {
		dev_dbg(&serial->dev->dev, "Empty rx buffer\n");
		return;
	}

	for (i = 0; (i + sizeof(*rxb)) <= urb->actual_length; i += sizeof(*rxb)) {
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
	struct usb_serial_port *port = urb->context;

	switch (urb->status) {
	case 0:
		/* processing continues once we receive UART_IIR_THRI. */
		return;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		/* this urb is terminated, clean up */
		dev_dbg(&port->dev,
			"%s - urb shutting down with status: %d\n",
			__func__, urb->status);
		break;
	default:
		dev_err_console(port,
				"%s - nonzero write bulk status received: %d\n",
				__func__, urb->status);
		break;
	}

	ch348_clear_write_state(port);
	usb_serial_port_softint(port);
}

static int ch348_write_config(struct usb_serial *serial, u8 cmd, u8 reg,
			      void *data, size_t len)
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

	config_port = serial->port[CH348_PORTNUM_CONFIG_WRITE];
	config_pipe = usb_sndbulkpipe(serial->dev,
				      config_port->bulk_out_endpointAddress);

	ret = usb_bulk_msg(serial->dev, config_pipe, buf, buf_len, NULL,
			   CH348_CMD_TIMEOUT);

	kfree(buf);

	return ret < 0 ? ret : 0;
}

static int ch348_write_config_be64(struct usb_serial *serial, u8 cmd, u8 reg,
				   u64 val)
{
	__be64 config_val = cpu_to_be64(val);

	return ch348_write_config(serial, cmd, reg, &config_val,
				  sizeof(config_val));
}

static int ch348_port_config(struct usb_serial_port *port, u8 cmd, u8 reg,
			     u8 control)
{
	if (port->port_number < 4)
		reg += 0x10 * port->port_number;
	else
		reg += 0x10 * (port->port_number - 4) + 0x08;

	return ch348_write_config(port->serial, cmd, reg, &control,
				  sizeof(control));
}

static int ch348_write(struct tty_struct *tty, struct usb_serial_port *port,
		       const unsigned char *buf, int count)
{
	int ret;

	if (!count)
		return 0;

	count = kfifo_in_locked(&port->write_fifo, buf, count, &port->lock);

	ret = ch348_write_start(port, GFP_ATOMIC);
	if (ret)
		return ret;

	return count;
}

static int ch348_set_modem_control(struct usb_serial_port *port, u8 mcr)
{
	struct ch348 *ch348 = usb_get_serial_data(port->serial);
	bool dtr, rts;
	int ret;

	/*
	 * Only the first four ports have the modem control pins routed outside
	 * the CH348Q package.
	 */
	if (ch348->package_type == CH348Q && port->port_number >= 4) {
		dev_dbg(&port->dev,
			"DTR/RTS is not supported on CH348Q port %u\n",
			port->port_number);
		return -ENOTSUPP;
	}

	dtr = !!(mcr & UART_MCR_DTR);
	rts = !!(mcr & UART_MCR_RTS);

	ret = ch348_port_config(port, CMD_W_BR, R_C4,
				dtr ? R_C4_DTR_ON : R_C4_DTR_OFF);
	if (ret) {
		dev_err(&port->dev, "Failed to set DTR = %s in R_C4: %d\n",
			str_on_off(dtr), ret);
		return ret;
	}

	ret = ch348_port_config(port, CMD_W_BR, R_C4,
				rts ? R_C4_RTS_ON : R_C4_RTS_OFF);
	if (ret) {
		dev_err(&port->dev, "Failed to set RTS = %s in R_C4: %d\n",
			str_on_off(rts), ret);
		return ret;
	}

	return 0;
}

static void ch348_set_termios(struct tty_struct *tty, struct usb_serial_port *port,
			      const struct ktermios *termios_old)
{
	struct ch348_config_data_init config = {};
	struct ktermios *termios = &tty->termios;
	int ret, portnum = port->port_number;
	speed_t	baudrate;

	/* Hardware flow control is supported by HW but not implemented yet */
	termios->c_cflag &= ~CRTSCTS;

	if (termios_old && !tty_termios_hw_change(termios, termios_old))
		return;

	baudrate = tty_termios_baud_rate(termios);
	if (!baudrate) {
		ch348_set_modem_control(port, 0);
		return;
	}

	/*
	 * The datasheet states that only baud rates in range of
	 * 1200..6000000 are supported. Tests with an oscilloscope
	 * confirm that even when configuring a baud rate slower than
	 * 1200 the output stays at around 1200 baud.
	 */
	baudrate = clamp(baudrate, 1200, 6000000);
	tty_termios_encode_baud_rate(termios, baudrate, baudrate);

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
		config.format = CH348_CONFIG_DATA_INIT_FORMAT_TWO_STOPBITS;
	else
		config.format = CH348_CONFIG_DATA_INIT_FORMAT_ONE_STOPBIT;

	if (baudrate >= 921600)
		config.recv_tmt = 5;
	else
		config.recv_tmt = (10000 * 15 / baudrate) + 1;

	ret = ch348_write_config(port->serial, CMD_WB_E | portnum, R_INIT,
				 &config, sizeof(config));
	if (ret < 0) {
		dev_err(&port->dev, "Failed to change line settings: %d\n",
			ret);
		if (termios_old)
			tty_termios_copy_hw(termios, termios_old);
	} else if (termios_old && (termios_old->c_cflag & CBAUD) == B0) {
		ch348_set_modem_control(port, UART_MCR_DTR | UART_MCR_RTS);
	}
}

static int ch348_break_ctl(struct tty_struct *tty, int on)
{
	struct usb_serial_port *port = tty->driver_data;
	int ret;

	ret = ch348_port_config(port, CMD_W_BR, R_C3,
				on ? R_C3_BREAK_ON : R_C3_BREAK_OFF);
	if (ret)
		dev_err(&port->dev, "Failed to set BREAK = %s in R_C3: %d\n",
			str_on_off(on), ret);

	return ret;
}

static void ch348_dtr_rts(struct usb_serial_port *port, int on)
{
	ch348_set_modem_control(port, on ? UART_MCR_DTR | UART_MCR_RTS : 0);
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
		dev_err(&port->dev, "Failed to set ACTIVATE in R_C2: %d\n",
			ret);
		goto err_kill_read_urbs;
	}

	ret = ch348_port_config(port, CMD_W_R, R_C4, R_C4_ACTIVATE);
	if (ret) {
		dev_err(&port->dev, "Failed to set ACTIVATE in R_C4: %d\n",
			ret);
		goto err_kill_read_urbs;
	}

	ret = ch348_port_config(port, CMD_W_R, UART_IER, UART_IER_RDI |
				UART_IER_THRI | UART_IER_RLSI | UART_IER_MSI);
	if (ret) {
		dev_err(&port->dev, "Failed to enable interrupts: %d\n", ret);
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
	int ret;

	ret = ch348_port_config(port, CMD_W_R, UART_IER, 0);
	if (ret)
		dev_dbg(&port->dev, "Failed to disable interrupts: %d\n", ret);

	scoped_guard(mutex, &ch348->open_ports_lock) {
		clear_bit(port->port_number, ch348->open_ports);

		if (bitmap_empty(ch348->open_ports, CH348_MAXPORT))
			ch348_kill_read_urbs(port->serial);
	}

	scoped_guard(spinlock_irqsave, &port->lock)
		kfifo_reset_out(&port->write_fifo);

	ch348_clear_write_state(port);

	usb_kill_urb(port->write_urb);
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

static int ch348_gpio_set_enabled(struct gpio_chip *gc, unsigned int offset,
				  bool enable)
{
	struct usb_serial *serial = gpiochip_get_data(gc);
	struct ch348 *ch348 = usb_get_serial_data(serial);
	u64 val;

	scoped_guard(mutex, &ch348->gpio_value_lock) {
		if (enable)
			set_bit(offset, ch348->gpio_enabled_mask);
		else
			clear_bit(offset, ch348->gpio_enabled_mask);

		bitmap_to_arr64(&val, ch348->gpio_enabled_mask, gc->ngpio);

		return ch348_write_config_be64(serial, R_MOD, R_IO_CE, val);
	}
}

static int ch348_gpio_request(struct gpio_chip *gc, unsigned int offset)
{
	return ch348_gpio_set_enabled(gc, offset, true);
}

static void ch348_gpio_free(struct gpio_chip *gc, unsigned int offset)
{
	ch348_gpio_set_enabled(gc, offset, false);
}

static int ch348_gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	struct usb_serial *serial = gpiochip_get_data(gc);
	struct ch348 *ch348 = usb_get_serial_data(serial);
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
	struct usb_serial *serial = gpiochip_get_data(gc);
	struct ch348 *ch348 = usb_get_serial_data(serial);
	u64 val;

	scoped_guard(mutex, &ch348->gpio_value_lock) {
		if (direction == GPIO_LINE_DIRECTION_OUT)
			set_bit(offset, ch348->gpio_dir_mask);
		else
			clear_bit(offset, ch348->gpio_dir_mask);

		bitmap_to_arr64(&val, ch348->gpio_dir_mask, gc->ngpio);

		return ch348_write_config_be64(serial, R_MOD, R_IO_CD, val);
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
	struct usb_serial *serial = gpiochip_get_data(gc);
	struct ch348 *ch348 = usb_get_serial_data(serial);
	int ret;

	if (ch348_gpios[offset].direction == CH348_GPIO_DIRECTION_INPUT_ONLY)
		return -EINVAL;

	ret = ch348_gpio_set_direction(gc, offset, GPIO_LINE_DIRECTION_OUT);
	if (ret)
		return ret;

	return ch348->gc.set(gc, offset, value);
}

static int ch348_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct usb_serial *serial = gpiochip_get_data(gc);
	struct ch348 *ch348 = usb_get_serial_data(serial);
	DECLARE_BITMAP(mask, CH348_NUM_GPIO) = { 0 };
	DECLARE_BITMAP(bits, CH348_NUM_GPIO) = { 0 };
	int ret;

	set_bit(offset, mask);

	ret = ch348->gc.get_multiple(gc, mask, bits);
	if (ret)
		return ret;

	return test_bit(offset, bits);
}

static int ch348_gpio_set(struct gpio_chip *gc, unsigned int offset, int value)
{
	struct usb_serial *serial = gpiochip_get_data(gc);
	struct ch348 *ch348 = usb_get_serial_data(serial);
	DECLARE_BITMAP(mask, CH348_NUM_GPIO) = { 0 };
	DECLARE_BITMAP(bits, CH348_NUM_GPIO) = { 0 };

	set_bit(offset, mask);

	if (value)
		set_bit(offset, bits);

	return ch348->gc.set_multiple(gc, mask, bits);
}

static int ch348_gpio_get_multiple(struct gpio_chip *gc, unsigned long *mask,
				   unsigned long *bits)
{
	unsigned long jiffies = msecs_to_jiffies(CH348_CMD_TIMEOUT);
	struct usb_serial *serial = gpiochip_get_data(gc);
	struct ch348 *ch348 = usb_get_serial_data(serial);
	int ret;

	scoped_guard(mutex, &ch348->gpio_in_lock) {
		reinit_completion(&ch348->gpio_in_completion);

		ret = ch348_write_config(serial, R_MOD, R_IO_CI, NULL, 0);
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
	struct usb_serial *serial = gpiochip_get_data(gc);
	struct ch348 *ch348 = usb_get_serial_data(serial);
	u64 val;

	scoped_guard(mutex, &ch348->gpio_value_lock) {
		bitmap_replace(ch348->gpio_out_mask, ch348->gpio_out_mask,
			       bits, mask, gc->ngpio);

		bitmap_to_arr64(&val, ch348->gpio_out_mask, gc->ngpio);

		return ch348_write_config_be64(serial, R_MOD, R_IO_CO, val);
	}
}

static int ch348_attach(struct usb_serial *serial)
{
	struct ch348 *ch348;
	unsigned int i;
	int ret;

	ch348 = kzalloc_obj(*ch348, GFP_KERNEL);
	if (!ch348)
		return -ENOMEM;

	usb_set_serial_data(serial, ch348);

	mutex_init(&ch348->open_ports_lock);
	mutex_init(&ch348->gpio_value_lock);
	mutex_init(&ch348->gpio_in_lock);

	init_completion(&ch348->gpio_in_completion);

	ret = ch348_detect_version(serial);
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
	ch348->gc.set = ch348_gpio_set;
	ch348->gc.get_multiple = ch348_gpio_get_multiple;
	ch348->gc.set_multiple = ch348_gpio_set_multiple;
	ch348->gc.owner = THIS_MODULE;
	ch348->gc.parent = &serial->dev->dev;
	ch348->gc.base = -1;
	ch348->gc.can_sleep = true;
	ch348->gc.names = ch348->gpio_names;

	ch348->gc.ngpio = ch348->package_type == CH348Q ? 12 : CH348_NUM_GPIO;
	ch348->gc.label = ch348->package_type == CH348Q ? "CH348Q" : "CH348L";

	ret = gpiochip_add_data(&ch348->gc, serial);
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

	mutex_destroy(&ch348->gpio_in_lock);
	mutex_destroy(&ch348->gpio_value_lock);
	mutex_destroy(&ch348->open_ports_lock);

	kfree(ch348);
}

static int ch348_calc_num_ports(struct usb_serial *serial,
				struct usb_serial_endpoints *epds)
{
	int i;

	/*
	 * Reserve a bulk out for each serial port plus an additional one
	 * (which is not registered as a serial port) for the config bulk out
	 * so each actual serial port has it's own write_urb that we can use
	 * for transmitting data.
	 */
	epds->num_bulk_out = CH348_PORTNUM_CONFIG_WRITE + 1;

	epds->bulk_out[CH348_PORTNUM_CONFIG_WRITE] = epds->bulk_out[1];

	for (i = 0; i < CH348_MAXPORT; ++i)
		epds->bulk_out[i] = epds->bulk_out[0];

	return CH348_MAXPORT;
}

static int ch348_suspend(struct usb_serial *serial, pm_message_t message)
{
	struct ch348 *ch348 = usb_get_serial_data(serial);
	unsigned int i;

	scoped_guard(mutex, &ch348->open_ports_lock) {
		if (bitmap_empty(ch348->open_ports, CH348_MAXPORT))
			return 0;

		ch348_kill_read_urbs(serial);

		for_each_set_bit(i, ch348->open_ports, CH348_MAXPORT) {
			usb_kill_urb(serial->port[i]->write_urb);
			ch348_clear_write_state(serial->port[i]);
		}
	}

	return 0;
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
			ret = ch348_write_start(serial->port[i], GFP_NOIO);
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
	.bulk_in_size =		512,
	.bulk_out_size =	512,
	.open =			ch348_open,
	.close =		ch348_close,
	.set_termios =		ch348_set_termios,
	.break_ctl =		ch348_break_ctl,
	.get_icount =		usb_serial_generic_get_icount,
	.dtr_rts =		ch348_dtr_rts,
	.process_read_urb =	ch348_process_read_urb,
	.write_bulk_callback =	ch348_write_bulk_callback,
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
MODULE_AUTHOR("Martin Blumenstingl <martin.blumenstingl@googlemail.com>");
MODULE_DESCRIPTION("USB CH348 Octo port serial converter driver");
MODULE_LICENSE("GPL");
