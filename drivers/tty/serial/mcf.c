// SPDX-License-Identifier: GPL-2.0+
/****************************************************************************/

/*
 *	mcf.c -- Freescale ColdFire UART driver
 *
 *	(C) Copyright 2003-2007, Greg Ungerer <gerg@uclinux.org>
 */

/****************************************************************************/

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/console.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <asm/coldfire.h>
#include <asm/mcfsim.h>
#include <asm/mcfuart.h>
#include <asm/nettel.h>

#include <linux/circ_buf.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>

/****************************************************************************/

/*
 *	Some boards implement the DTR/DCD lines using GPIO lines, most
 *	don't. Dummy out the access macros for those that don't. Those
 *	that do should define these macros somewhere in there board
 *	specific inlude files.
 */
#if !defined(mcf_getppdcd)
#define	mcf_getppdcd(p)		(1)
#endif
#if !defined(mcf_getppdtr)
#define	mcf_getppdtr(p)		(1)
#endif
#if !defined(mcf_setppdtr)
#define	mcf_setppdtr(p, v)	do { } while (0)
#endif

/****************************************************************************/

/*
 *	Local per-uart structure.
 */
struct mcf_uart {
	struct uart_port	port;
	unsigned int		sigs;		/* Local copy of line sigs */
	unsigned char		imr;		/* Local IMR mirror */

	/* DMA fields */
	unsigned int		dma_is_enabled:1;
	struct dma_chan		*dma_chan_rx, *dma_chan_tx;
	void			*rx_buf;
	struct circ_buf		rx_ring;
	unsigned int		rx_buf_size;
	unsigned int		rx_period_length;
	unsigned int		rx_periods;
	dma_cookie_t		rx_cookie;
	dma_addr_t		dma_buf;	/* SW-FIFO (DMA destination) */
	struct dma_async_tx_descriptor *desc;
	struct hrtimer		rx_timer;
	bool			rx_dma_running;
	unsigned int		last_r_bytes;
};

#define MCFUART_DMA_RXFIFOSIZE	128

/****************************************************************************/

static unsigned int mcf_tx_empty(struct uart_port *port)
{
	return (readb(port->membase + MCFUART_USR) & MCFUART_USR_TXEMPTY) ?
		TIOCSER_TEMT : 0;
}

/****************************************************************************/

static unsigned int mcf_get_mctrl(struct uart_port *port)
{
	struct mcf_uart *pp = container_of(port, struct mcf_uart, port);
	unsigned int sigs;

	sigs = (readb(port->membase + MCFUART_UIPR) & MCFUART_UIPR_CTS) ?
		0 : TIOCM_CTS;
	sigs |= (pp->sigs & TIOCM_RTS);
	sigs |= (mcf_getppdcd(port->line) ? TIOCM_CD : 0);
	sigs |= (mcf_getppdtr(port->line) ? TIOCM_DTR : 0);

	return sigs;
}

/****************************************************************************/

static void mcf_set_mctrl(struct uart_port *port, unsigned int sigs)
{
	struct mcf_uart *pp = container_of(port, struct mcf_uart, port);

	pp->sigs = sigs;
	mcf_setppdtr(port->line, (sigs & TIOCM_DTR));
	if (sigs & TIOCM_RTS)
		writeb(MCFUART_UOP_RTS, port->membase + MCFUART_UOP1);
	else
		writeb(MCFUART_UOP_RTS, port->membase + MCFUART_UOP0);
}

/****************************************************************************/

static void mcf_start_tx(struct uart_port *port)
{
	struct mcf_uart *pp = container_of(port, struct mcf_uart, port);

	if (port->rs485.flags & SER_RS485_ENABLED) {
		/* Enable Transmitter */
		writeb(MCFUART_UCR_TXENABLE, port->membase + MCFUART_UCR);
		/* Manually assert RTS */
		writeb(MCFUART_UOP_RTS, port->membase + MCFUART_UOP1);
	}
	pp->imr |= MCFUART_UIR_TXREADY;
	writeb(pp->imr, port->membase + MCFUART_UIMR);
}

/****************************************************************************/

static void mcf_stop_tx(struct uart_port *port)
{
	struct mcf_uart *pp = container_of(port, struct mcf_uart, port);

	pp->imr &= ~MCFUART_UIR_TXREADY;
	writeb(pp->imr, port->membase + MCFUART_UIMR);
}

/****************************************************************************/

static void mcf_stop_rx(struct uart_port *port)
{
	struct mcf_uart *pp = container_of(port, struct mcf_uart, port);

	pp->imr &= ~MCFUART_UIR_RXREADY;
	writeb(pp->imr, port->membase + MCFUART_UIMR);
}

/****************************************************************************/

static void mcf_break_ctl(struct uart_port *port, int break_state)
{
	unsigned long flags;

	uart_port_lock_irqsave(port, &flags);
	if (break_state == -1)
		writeb(MCFUART_UCR_CMDBREAKSTART, port->membase + MCFUART_UCR);
	else
		writeb(MCFUART_UCR_CMDBREAKSTOP, port->membase + MCFUART_UCR);
	uart_port_unlock_irqrestore(port, flags);
}

/****************************************************************************/

#define TOTAL_SIZE 256
#define RX_DMA_TIMER_INTERVAL	11

void mcf_uart_dma_rx_callback(void *data);

static int mcf_copy_dma_to_tty(struct mcf_uart *sport)
{
	struct uart_port *port = &sport->port;
	struct tty_port *tport = &port->state->port;
	struct circ_buf *rx_ring = &sport->rx_ring;
	unsigned int w_bytes, r_bytes;
	int residue = __raw_readw(0xfc0450d4);
	int ret = 0;

	/* Calculate the number of bytes ready and adjust head/tail */
	rx_ring->head = sport->rx_period_length - residue;
	r_bytes = CIRC_CNT(rx_ring->head, rx_ring->tail, sport->rx_period_length);

	if ((r_bytes != 0) && (sport->rx_dma_running == false)) {
		//trace_printk("DMA RX started\n");
		sport->rx_dma_running = true;
		goto done;
	}

	if ((sport->last_r_bytes - r_bytes == 0) && (sport->rx_dma_running == true))
	{
		/* Copy all the received data from the ring buffer to the tty buffer */
		dma_sync_single_for_cpu(sport->port.dev, sport->dma_buf, sport->rx_buf_size, DMA_FROM_DEVICE);
		w_bytes = tty_insert_flip_string(tport, rx_ring->buf + rx_ring->tail, r_bytes);
		dma_sync_single_for_device(sport->port.dev, sport->dma_buf, sport->rx_buf_size, DMA_FROM_DEVICE);

		rx_ring->tail = (rx_ring->tail + w_bytes) & (sport->rx_period_length - 1);

		sport->port.icount.rx += w_bytes;

		tty_flip_buffer_push(tport);
		ret = w_bytes;
	}

done:
	sport->last_r_bytes = r_bytes;
	return ret;
}

static int mcf_next_xfer_rx_dma(struct mcf_uart *sport)
{
	struct dma_chan	*chan = sport->dma_chan_rx;
	struct device *dev = sport->port.dev;

	sport->rx_ring.head = 0;
	sport->rx_ring.tail = 0;
	sport->rx_ring.buf = sport->rx_buf;
	sport->rx_dma_running = false;
	sport->last_r_bytes = 0;

	sport->desc =  dmaengine_prep_slave_single(chan, sport->dma_buf, sport->rx_period_length,
					    DMA_DEV_TO_MEM, DMA_PREP_INTERRUPT | DMA_CTRL_REUSE);
	if (!sport->desc) {
		dev_err(dev, "DMA prep cyclic error\n");
		devm_kfree(dev, sport->rx_buf);
		return -EINVAL;
	}

	sport->desc->callback = mcf_uart_dma_rx_callback;
	sport->desc->callback_param = sport;

	sport->rx_cookie = dmaengine_submit(sport->desc);
	if (dma_submit_error(sport->rx_cookie)) {
		dev_err(dev, "DMA submit error\n");
		devm_kfree(dev, sport->rx_buf);
		return -EINVAL;
	}
	dma_async_issue_pending(chan);
	//trace_printk("Prepared DMA\n");

	return 0;
}

static enum hrtimer_restart mcf_rx_thread(struct hrtimer *timer)
{
	struct mcf_uart *sport = container_of(timer, struct mcf_uart, rx_timer);
	struct dma_chan *chan = sport->dma_chan_rx;
//	struct sched_param sp = { .sched_priority = 93 };
	int w_bytes;

//	sched_setscheduler(current, SCHED_FIFO, &sp);

	dmaengine_pause(chan);
	w_bytes = mcf_copy_dma_to_tty(sport);

	if (w_bytes == 0) {
		dmaengine_resume(chan);
	}
	else {
		//trace_printk("Copied %d bytes to tty\n", w_bytes);
		dmaengine_terminate_async(chan);
		mcf_next_xfer_rx_dma(sport);
	}

	hrtimer_forward_now(&sport->rx_timer, ms_to_ktime(RX_DMA_TIMER_INTERVAL));
	return HRTIMER_RESTART;
}

void mcf_uart_dma_rx_callback(void *data)
{
	struct mcf_uart *sport = data;
	struct dma_chan *chan = sport->dma_chan_rx;
	struct dma_tx_state state;
	enum dma_status status;

	//trace_printk("IRQ callback\n");
	status = dmaengine_tx_status(chan, sport->rx_cookie, &state);

	if (status == DMA_ERROR) {
		trace_printk("error: %08x\n", __raw_readl(0xfc044004));
		uart_port_lock(&sport->port);
		// imx_uart_clear_rx_errors(sport);
		uart_port_unlock(&sport->port);
		goto done;
	}

	mcf_copy_dma_to_tty(sport);

done:
	/* Re-enable the DMA channel */
	if (status == DMA_COMPLETE)
		mcf_next_xfer_rx_dma(sport);

	return;
}

static int mcf_dma_setup(struct uart_port *port)
{
	struct mcf_uart *sport = container_of(port, struct mcf_uart, port);
	struct dma_chan *chan;
	struct dma_slave_config slave_config = {};
	int ret;

	/* Prepare for RX : */
	chan = dma_request_chan(port->dev, "rx");
	if (IS_ERR(chan)) {
		sport->dma_chan_rx = NULL;
		ret = PTR_ERR(chan);
		dev_err(port->dev, "cannot get the DMA channel: %d\n", ret);
		goto err;
	}
	sport->dma_chan_rx = chan;
	printk("%s: Chan %d requested\n", __func__, chan->chan_id);

	slave_config.direction = DMA_DEV_TO_MEM;
	slave_config.src_addr = (phys_addr_t) (port->membase + MCFUART_URB);
	slave_config.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	slave_config.src_maxburst = 1;
	slave_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	slave_config.dst_maxburst = 1;

	ret = dmaengine_slave_config(sport->dma_chan_rx, &slave_config);
	if (ret) {
		dev_err(port->dev, "error in RX dma configuration.\n");
		goto err;
	}

	sport->rx_period_length = TOTAL_SIZE / 2;
	sport->rx_buf_size = TOTAL_SIZE;
	sport->rx_buf = devm_kzalloc(port->dev, sport->rx_buf_size, GFP_DMA);
	if (!sport->rx_buf) {
		ret = -ENOMEM;
		goto err;
	}
	sport->dma_buf = virt_to_phys(sport->rx_buf);

err:
	return ret;
}

static int mcf_startup(struct uart_port *port)
{
	struct mcf_uart *pp = container_of(port, struct mcf_uart, port);
	unsigned long flags;

	uart_port_lock_irqsave(port, &flags);

	/* Reset UART, get it into known state... */
	writeb(MCFUART_UCR_CMDRESETRX, port->membase + MCFUART_UCR);
	writeb(MCFUART_UCR_CMDRESETTX, port->membase + MCFUART_UCR);

	/* Enable the UART transmitter and receiver */
	writeb(MCFUART_UCR_RXENABLE | MCFUART_UCR_TXENABLE,
		port->membase + MCFUART_UCR);

	if (port->line != 2) {
		/* Enable RX interrupts now */
		pp->imr = MCFUART_UIR_RXREADY;
		writeb(pp->imr, port->membase + MCFUART_UIMR);
	}

	if (port->line == 2) {
		hrtimer_start(&pp->rx_timer, ms_to_ktime(RX_DMA_TIMER_INTERVAL), HRTIMER_MODE_REL);
		mcf_next_xfer_rx_dma(pp);
	}

	uart_port_unlock_irqrestore(port, flags);

	return 0;
}

/****************************************************************************/

static void mcf_shutdown(struct uart_port *port)
{
	struct mcf_uart *pp = container_of(port, struct mcf_uart, port);
	unsigned long flags;

	uart_port_lock_irqsave(port, &flags);

	hrtimer_cancel(&pp->rx_timer);

	/* Disable all interrupts now */
	pp->imr = 0;
	writeb(pp->imr, port->membase + MCFUART_UIMR);

	/* Disable UART transmitter and receiver */
	writeb(MCFUART_UCR_CMDRESETRX, port->membase + MCFUART_UCR);
	writeb(MCFUART_UCR_CMDRESETTX, port->membase + MCFUART_UCR);

	uart_port_unlock_irqrestore(port, flags);
}

/****************************************************************************/

static void mcf_set_termios(struct uart_port *port, struct ktermios *termios,
			    const struct ktermios *old)
{
	unsigned long flags;
	unsigned int baud, baudclk;
#if defined(CONFIG_M5272)
	unsigned int baudfr;
#endif
	unsigned char mr1, mr2;

	baud = uart_get_baud_rate(port, termios, old, 0, 230400);
#if defined(CONFIG_M5272)
	baudclk = (MCF_BUSCLK / baud) / 32;
	baudfr = (((MCF_BUSCLK / baud) + 1) / 2) % 16;
#else
	baudclk = ((MCF_BUSCLK / baud) + 16) / 32;
#endif

	mr1 = MCFUART_MR1_RXIRQRDY | MCFUART_MR1_RXERRCHAR;
	mr2 = 0;

	switch (termios->c_cflag & CSIZE) {
	case CS5: mr1 |= MCFUART_MR1_CS5; break;
	case CS6: mr1 |= MCFUART_MR1_CS6; break;
	case CS7: mr1 |= MCFUART_MR1_CS7; break;
	case CS8:
	default:  mr1 |= MCFUART_MR1_CS8; break;
	}

	if (termios->c_cflag & PARENB) {
		if (termios->c_cflag & CMSPAR) {
			if (termios->c_cflag & PARODD)
				mr1 |= MCFUART_MR1_PARITYMARK;
			else
				mr1 |= MCFUART_MR1_PARITYSPACE;
		} else {
			if (termios->c_cflag & PARODD)
				mr1 |= MCFUART_MR1_PARITYODD;
			else
				mr1 |= MCFUART_MR1_PARITYEVEN;
		}
	} else {
		mr1 |= MCFUART_MR1_PARITYNONE;
	}

	/*
	 * FIXME: port->read_status_mask and port->ignore_status_mask
	 * need to be initialized based on termios settings for
	 * INPCK, IGNBRK, IGNPAR, PARMRK, BRKINT
	 */

	if (termios->c_cflag & CSTOPB)
		mr2 |= MCFUART_MR2_STOP2;
	else
		mr2 |= MCFUART_MR2_STOP1;

	if (termios->c_cflag & CRTSCTS) {
		mr1 |= MCFUART_MR1_RXRTS;
		mr2 |= MCFUART_MR2_TXCTS;
	}

	uart_port_lock_irqsave(port, &flags);
	if (port->rs485.flags & SER_RS485_ENABLED) {
		dev_dbg(port->dev, "Setting UART to RS485\n");
		mr2 |= MCFUART_MR2_TXRTS;
	}

	uart_update_timeout(port, termios->c_cflag, baud);
	writeb(MCFUART_UCR_CMDRESETRX, port->membase + MCFUART_UCR);
	writeb(MCFUART_UCR_CMDRESETTX, port->membase + MCFUART_UCR);
	writeb(MCFUART_UCR_CMDRESETMRPTR, port->membase + MCFUART_UCR);
	writeb(mr1, port->membase + MCFUART_UMR);
	writeb(mr2, port->membase + MCFUART_UMR);
	writeb((baudclk & 0xff00) >> 8, port->membase + MCFUART_UBG1);
	writeb((baudclk & 0xff), port->membase + MCFUART_UBG2);
#if defined(CONFIG_M5272)
	writeb((baudfr & 0x0f), port->membase + MCFUART_UFPD);
#endif
	writeb(MCFUART_UCSR_RXCLKTIMER | MCFUART_UCSR_TXCLKTIMER,
		port->membase + MCFUART_UCSR);
	writeb(MCFUART_UCR_RXENABLE | MCFUART_UCR_TXENABLE,
		port->membase + MCFUART_UCR);
	uart_port_unlock_irqrestore(port, flags);
}

/****************************************************************************/

static inline void mcf_rx_one_char(struct uart_port *port,
				   unsigned char     status,
				   unsigned char     ch)
{
	unsigned char flag;

	flag = TTY_NORMAL;
	if (status & MCFUART_USR_RXERR) {
		writeb(MCFUART_UCR_CMDRESETERR,
			port->membase + MCFUART_UCR);

		if (status & MCFUART_USR_RXBREAK) {
			port->icount.brk++;
			if (uart_handle_break(port))
				return;
		} else if (status & MCFUART_USR_RXPARITY) {
			port->icount.parity++;
		} else if (status & MCFUART_USR_RXOVERRUN) {
			port->icount.overrun++;
		} else if (status & MCFUART_USR_RXFRAMING) {
			port->icount.frame++;
		}

		status &= port->read_status_mask;

		if (status & MCFUART_USR_RXBREAK)
			flag = TTY_BREAK;
		else if (status & MCFUART_USR_RXPARITY)
			flag = TTY_PARITY;
		else if (status & MCFUART_USR_RXFRAMING)
			flag = TTY_FRAME;
	}

	if (uart_handle_sysrq_char(port, ch))
		return;
	uart_insert_char(port, status, MCFUART_USR_RXOVERRUN, ch, flag);
}

static void mcf_rx_chars(struct mcf_uart *pp)
{
	struct uart_port *port = &pp->port;
	u8 status, ch;

	while ((status = readb(port->membase + MCFUART_USR)) & MCFUART_USR_RXREADY) {
		ch = readb(port->membase + MCFUART_URB);
		port->icount.rx++;

		mcf_rx_one_char(port, status, ch);
	}

	tty_flip_buffer_push(&port->state->port);
}

/****************************************************************************/

static void mcf_tx_chars(struct mcf_uart *pp)
{
	struct uart_port *port = &pp->port;
	bool pending;
	u8 ch;

	pending = uart_port_tx(port, ch,
		readb(port->membase + MCFUART_USR) & MCFUART_USR_TXREADY,
		writeb(ch, port->membase + MCFUART_UTB));

	/* Disable TX to negate RTS automatically */
	if (!pending && (port->rs485.flags & SER_RS485_ENABLED))
		writeb(MCFUART_UCR_TXDISABLE, port->membase + MCFUART_UCR);
}

/****************************************************************************/

static irqreturn_t mcf_interrupt(int irq, void *data)
{
	struct uart_port *port = data;
	struct mcf_uart *pp = container_of(port, struct mcf_uart, port);
	unsigned int isr;
	irqreturn_t ret = IRQ_NONE;

	isr = readb(port->membase + MCFUART_UISR) & pp->imr;

	uart_port_lock(port);
	if (isr & MCFUART_UIR_RXREADY) {
		mcf_rx_chars(pp);
		ret = IRQ_HANDLED;
	}
	if (isr & MCFUART_UIR_TXREADY) {
		mcf_tx_chars(pp);
		ret = IRQ_HANDLED;
	}
	uart_port_unlock(port);

	return ret;
}

/****************************************************************************/

static void mcf_config_port(struct uart_port *port, int flags)
{
	port->type = PORT_MCF;
	port->fifosize = MCFUART_TXFIFOSIZE;

	/* Clear mask, so no surprise interrupts. */
	writeb(0, port->membase + MCFUART_UIMR);

	if (request_irq(port->irq, mcf_interrupt, 0, "UART", port))
		printk(KERN_ERR "MCF: unable to attach ColdFire UART %d "
			"interrupt vector=%d\n", port->line, port->irq);
}

/****************************************************************************/

static const char *mcf_type(struct uart_port *port)
{
	return (port->type == PORT_MCF) ? "ColdFire UART" : NULL;
}

/****************************************************************************/

static int mcf_request_port(struct uart_port *port)
{
	/* UARTs always present */
	return 0;
}

/****************************************************************************/

static void mcf_release_port(struct uart_port *port)
{
	/* Nothing to release... */
}

/****************************************************************************/

static int mcf_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	if ((ser->type != PORT_UNKNOWN) && (ser->type != PORT_MCF))
		return -EINVAL;
	return 0;
}

/****************************************************************************/

/* Enable or disable the RS485 support */
static int mcf_config_rs485(struct uart_port *port, struct ktermios *termios,
			    struct serial_rs485 *rs485)
{
	unsigned char mr1, mr2;

	/* Get mode registers */
	mr1 = readb(port->membase + MCFUART_UMR);
	mr2 = readb(port->membase + MCFUART_UMR);
	if (rs485->flags & SER_RS485_ENABLED) {
		dev_dbg(port->dev, "Setting UART to RS485\n");
		/* Automatically negate RTS after TX completes */
		mr2 |= MCFUART_MR2_TXRTS;
	} else {
		dev_dbg(port->dev, "Setting UART to RS232\n");
		mr2 &= ~MCFUART_MR2_TXRTS;
	}
	writeb(mr1, port->membase + MCFUART_UMR);
	writeb(mr2, port->membase + MCFUART_UMR);

	return 0;
}

static const struct serial_rs485 mcf_rs485_supported = {
	.flags = SER_RS485_ENABLED | SER_RS485_RTS_AFTER_SEND,
};

/****************************************************************************/

/*
 *	Define the basic serial functions we support.
 */
static const struct uart_ops mcf_uart_ops = {
	.tx_empty	= mcf_tx_empty,
	.get_mctrl	= mcf_get_mctrl,
	.set_mctrl	= mcf_set_mctrl,
	.start_tx	= mcf_start_tx,
	.stop_tx	= mcf_stop_tx,
	.stop_rx	= mcf_stop_rx,
	.break_ctl	= mcf_break_ctl,
	.startup	= mcf_startup,
	.shutdown	= mcf_shutdown,
	.set_termios	= mcf_set_termios,
	.type		= mcf_type,
	.request_port	= mcf_request_port,
	.release_port	= mcf_release_port,
	.config_port	= mcf_config_port,
	.verify_port	= mcf_verify_port,
};

static struct mcf_uart mcf_ports[10];

#define	MCF_MAXPORTS	ARRAY_SIZE(mcf_ports)

/****************************************************************************/
#if defined(CONFIG_SERIAL_MCF_CONSOLE)
/****************************************************************************/

static void mcf_console_putc(struct console *co, const char c)
{
	struct uart_port *port = &(mcf_ports + co->index)->port;
	int i;

	for (i = 0; (i < 0x10000); i++) {
		if (readb(port->membase + MCFUART_USR) & MCFUART_USR_TXREADY)
			break;
	}
	writeb(c, port->membase + MCFUART_UTB);
	for (i = 0; (i < 0x10000); i++) {
		if (readb(port->membase + MCFUART_USR) & MCFUART_USR_TXREADY)
			break;
	}
}

/****************************************************************************/

static void mcf_console_write(struct console *co, const char *s, unsigned int count)
{
	for (; (count); count--, s++) {
		mcf_console_putc(co, *s);
		if (*s == '\n')
			mcf_console_putc(co, '\r');
	}
}

/****************************************************************************/

static int __init mcf_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud = CONFIG_SERIAL_MCF_BAUDRATE;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	if ((co->index < 0) || (co->index >= MCF_MAXPORTS))
		co->index = 0;
	port = &mcf_ports[co->index].port;
	if (port->membase == 0)
		return -ENODEV;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(port, co, baud, parity, bits, flow);
}

/****************************************************************************/

static struct uart_driver mcf_driver;

static struct console mcf_console = {
	.name		= "ttyS",
	.write		= mcf_console_write,
	.device		= uart_console_device,
	.setup		= mcf_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &mcf_driver,
};

static int __init mcf_console_init(void)
{
	register_console(&mcf_console);
	return 0;
}

console_initcall(mcf_console_init);

#define	MCF_CONSOLE	&mcf_console

/****************************************************************************/
#else
/****************************************************************************/

#define	MCF_CONSOLE	NULL

/****************************************************************************/
#endif /* CONFIG_SERIAL_MCF_CONSOLE */
/****************************************************************************/

/*
 *	Define the mcf UART driver structure.
 */
static struct uart_driver mcf_driver = {
	.owner		= THIS_MODULE,
	.driver_name	= "mcf",
	.dev_name	= "ttyS",
	.major		= TTY_MAJOR,
	.minor		= 64,
	.nr		= MCF_MAXPORTS,
	.cons		= MCF_CONSOLE,
};

/****************************************************************************/

static int mcf_probe(struct platform_device *pdev)
{
	struct uart_port *port;
	struct mcf_uart *pp;
	struct resource *res;
	void __iomem *base;
	int id = pdev->id;

	if (id == -1 || id >= MCF_MAXPORTS) {
		dev_err(&pdev->dev, "uart%d out of range\n",
			id);
		return -EINVAL;
	}

	port = &mcf_ports[id].port;
	port->line = id;

	base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	port->mapbase = res->start;
	port->membase = base;

	port->irq = platform_get_irq(pdev, 0);
	if (port->irq < 0)
		return port->irq;

	port->type = PORT_MCF;
	port->dev = &pdev->dev;
	port->iotype = SERIAL_IO_MEM;
	port->uartclk = MCF_BUSCLK;
	port->ops = &mcf_uart_ops;
	port->flags = UPF_BOOT_AUTOCONF;
	port->rs485_config = mcf_config_rs485;
	port->rs485_supported = mcf_rs485_supported;
	port->has_sysrq = IS_ENABLED(CONFIG_SERIAL_MCF_CONSOLE);

	pp = container_of(port, struct mcf_uart, port);
	pp->dma_chan_rx = NULL;

	mcf_dma_setup(port);

	if (port->line == 2) {
		hrtimer_init(&pp->rx_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		pp->rx_timer.function = mcf_rx_thread;
	}

	uart_add_one_port(&mcf_driver, port);

	return 0;
}

/****************************************************************************/

static void mcf_remove(struct platform_device *pdev)
{
	struct uart_port *port;
	int id = pdev->id;

	port = &mcf_ports[id].port;
	if (port)
		uart_remove_one_port(&mcf_driver, port);
}

/****************************************************************************/

static struct platform_driver mcf_platform_driver = {
	.probe		= mcf_probe,
	.remove		= mcf_remove,
	.driver		= {
		.name	= "mcfuart",
	},
};

/****************************************************************************/

static int __init mcf_init(void)
{
	int rc;

	printk("ColdFire internal UART serial driver\n");

	rc = uart_register_driver(&mcf_driver);
	if (rc)
		return rc;
	rc = platform_driver_register(&mcf_platform_driver);
	if (rc) {
		uart_unregister_driver(&mcf_driver);
		return rc;
	}
	return 0;
}

/****************************************************************************/

static void __exit mcf_exit(void)
{
	platform_driver_unregister(&mcf_platform_driver);
	uart_unregister_driver(&mcf_driver);
}

/****************************************************************************/

module_init(mcf_init);
module_exit(mcf_exit);

MODULE_AUTHOR("Greg Ungerer <gerg@uclinux.org>");
MODULE_DESCRIPTION("Freescale ColdFire UART driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:mcfuart");

/****************************************************************************/
