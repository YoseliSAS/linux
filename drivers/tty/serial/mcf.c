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
	unsigned int		dma_is_rxing:1;
	unsigned int		dma_is_txing:1;
	struct dma_chan		*dma_chan_rx, *dma_chan_tx;
	struct scatterlist	rx_sgl, tx_sgl[2];
	void			*rx_buf;
	struct circ_buf		rx_ring;
	unsigned int		rx_buf_size;
	unsigned int		rx_period_length;
	unsigned int		rx_periods;
	dma_cookie_t		rx_cookie;
	unsigned int		tx_bytes;
	unsigned int		dma_tx_nents;
	unsigned int            saved_reg[10];
	bool			context_saved;
	unsigned int		dma_chan_id;	/* DMA channel to request */
	unsigned char		*dma_buf;	/* SW-FIFO (DMA destination) */
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

static void mcf_uart_dma_rx_callback(void *data)
{
	struct mcf_uart *sport = data;
	struct uart_port *port = &sport->port;
	struct dma_chan	*chan = sport->dma_chan_rx;
	struct scatterlist *sgl = &sport->rx_sgl;
	struct dma_tx_state state;
	struct circ_buf *rx_ring = &sport->rx_ring;
	enum dma_status status;
	unsigned int w_bytes = 0;
	unsigned int r_bytes;
	unsigned int bd_size;

	status = dmaengine_tx_status(chan, sport->rx_cookie, &state);

	if (status == DMA_ERROR) {
		uart_port_lock(&sport->port);
		//mcf_uart_clear_rx_errors(sport);
		dev_err(port->dev, "DMA error !\n");
		uart_port_unlock(&sport->port);
		return;
	}

	trace_printk("We could get a DMA byte now !\n");
}

/* Default RX DMA buffer configuration */
#define RX_DMA_PERIODS		16
#define RX_DMA_PERIOD_LEN	(PAGE_SIZE / 4)

static int mcf_dma_setup(struct uart_port *port)
{
	struct mcf_uart *pp = container_of(port, struct mcf_uart, port);
	struct dma_chan *chan;
	struct dma_slave_config slave_config = {};
	int ret;

	/* Prepare for RX : */
	chan = dma_request_chan(port->dev, "rx");
	if (IS_ERR(chan)) {
		pp->dma_chan_rx = NULL;
		ret = PTR_ERR(chan);
		dev_err(port->dev, "cannot get the DMA channel: %d\n", ret);
		goto err;
	}
	pp->dma_chan_rx = chan;
	printk("%s: Chan %d requested\n", __func__, chan->chan_id);

	slave_config.direction = DMA_DEV_TO_MEM;
	slave_config.src_addr = (phys_addr_t) (port->membase + MCFUART_URB);
	slave_config.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;

	ret = dmaengine_slave_config(pp->dma_chan_rx, &slave_config);
	if (ret) {
		dev_err(port->dev, "error in RX dma configuration.\n");
		goto err;
	}

	pp->rx_period_length = RX_DMA_PERIOD_LEN;
	pp->rx_periods = RX_DMA_PERIODS;
	pp->rx_buf_size = pp->rx_period_length * pp->rx_periods;
	pp->rx_buf = devm_kzalloc(port->dev, pp->rx_buf_size, GFP_KERNEL);
	if (!pp->rx_buf) {
		ret = -ENOMEM;
		goto err;
	}
	pp->rx_ring.buf = pp->rx_buf;

err:
	return ret;
}

static int mcf_start_rx_dma(struct uart_port *port)
{
	struct mcf_uart *sport = container_of(port, struct mcf_uart, port);
	struct scatterlist *sgl = &sport->rx_sgl;
	struct dma_chan	*chan = sport->dma_chan_rx;
	struct device *dev = port->dev;
	struct dma_async_tx_descriptor *desc;
	int ret;

	dev_err(dev, "We are on line %dfor device %s\n", port->port_id, dev_name(dev));
	sport->rx_ring.head = 0;
	sport->rx_ring.tail = 0;

	sg_init_one(sgl, sport->rx_buf, sport->rx_buf_size);
	ret = dma_map_sg(dev, sgl, 1, DMA_FROM_DEVICE);
	if (ret == 0) {
		dev_err(dev, "DMA mapping error for RX.\n");
		return -EINVAL;
	}

	desc = dmaengine_prep_dma_cyclic(chan, sg_dma_address(sgl),
		sg_dma_len(sgl), sg_dma_len(sgl) / sport->rx_periods,
		DMA_DEV_TO_MEM, DMA_PREP_INTERRUPT);

	if (!desc) {
		dma_unmap_sg(dev, sgl, 1, DMA_FROM_DEVICE);
		dev_err(dev, "We cannot prepare for the RX slave dma!\n");
		return -EINVAL;
	}
	desc->callback = mcf_uart_dma_rx_callback;
	desc->callback_param = sport;

	dev_dbg(dev, "RX: prepare for the DMA.\n");
	sport->dma_is_rxing = 1;
	sport->rx_cookie = dmaengine_submit(desc);
	dma_async_issue_pending(chan);
	return 0;
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

	/* Enable RX interrupts now */
	pp->imr = MCFUART_UIR_RXREADY;
	writeb(pp->imr, port->membase + MCFUART_UIMR);

	if (port->port_id == 2)
		mcf_start_rx_dma(port);

	uart_port_unlock_irqrestore(port, flags);

	return 0;
}

/****************************************************************************/

static void mcf_shutdown(struct uart_port *port)
{
	struct mcf_uart *pp = container_of(port, struct mcf_uart, port);
	unsigned long flags;

	uart_port_lock_irqsave(port, &flags);

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

#if 1

static void mcf_rx_handler(struct mcf_uart *pp)
{
	struct uart_port *port = &pp->port;
	int count = 0;
	u8 status;
	unsigned char flag;
	struct char_buf {
		u8 ch;
		u8 flag;
		u8 status;
	};
	struct char_buf buf[256];

	status = readb(port->membase + MCFUART_USR);
	do {
		
			buf[count].ch = readb(port->membase + MCFUART_URB);

			flag = TTY_NORMAL;
			buf[count].flag = TTY_NORMAL;
			buf[count].status = 0;
			count++;
	} while ((status = readb(port->membase + MCFUART_USR)) & MCFUART_USR_RXREADY);
#if 0
	if (status & MCFUART_USR_RXERR) {
		writeb(MCFUART_UCR_CMDRESETERR,
			port->membase + MCFUART_UCR);

		status &= port->read_status_mask;

		if (status & MCFUART_USR_RXBREAK)
			flag = TTY_BREAK;
		else if (status & MCFUART_USR_RXPARITY)
			flag = TTY_PARITY;
		else if (status & MCFUART_USR_RXFRAMING)
			flag = TTY_FRAME;
	}
#endif
	for (int i = 0 ; i < count ; i++)
		uart_insert_char(port, buf[i].status, MCFUART_USR_RXOVERRUN, buf[i].ch, buf[i].flag);
	
	port->icount.rx += count;
	trace_printk("Received %d chars on port %d\n", count, port->port_id);

	tty_flip_buffer_push(&port->state->port);
}
#else

static void mcf_rx_handler(unsigned long data)
{
    struct uart_port *port = (struct uart_port *)data;
    struct mcf_uart *pp = container_of(port, struct mcf_uart, port);
    int count = port->icount.rx;
    u8 status, ch;
    int char_count = 0;
    const int max_chars = 4;
    const u64 max_delay_ns = max_chars*100;  // 900 µs en nanosecondes
    u64 start_time_ns = ktime_get_ns();  // Sauvegarde du temps de départ
    bool stop = false;

    do {
        int read_chars = 0;

        // Vérifier si le FIFO est plein avec le bit MCFUART_USR_RXFULL
        if ((status = readb(port->membase + MCFUART_USR)) & MCFUART_USR_RXREADY) {
            do {
                ch = readb(port->membase + MCFUART_URB);
                port->icount.rx++;
                mcf_rx_one_char(port, status, ch);
                read_chars++;
                char_count++;

                // Sortir si on atteint le nombre total de caractères
                if (char_count >= max_chars) {
                    stop = true;
                    break;
                }
            } while ((status & MCFUART_USR_RXFULL) && read_chars < max_chars);

            // Si le FIFO était plein, on applique un délai plus court
            if (status & MCFUART_USR_RXFULL && !stop) {
                udelay(40);  // Petit délai si le FIFO était plein
            } else if (!stop) {
                udelay(80);  // Délai normal
            }
        } else {
            stop = true;  // Arrêter si aucun caractère n'est prêt à être lu
        }

        // Sortir si le délai imparti est dépassé
        if ((ktime_get_ns() - start_time_ns) > max_delay_ns) {
            stop = true;
        }

    } while (!stop);

    trace_printk("Received %d chars on port %d\n", port->icount.rx - count, port->port_id);

    tty_flip_buffer_push(&port->state->port);
}


#endif
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
	u32 rx_count = port->icount.rx;

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

	if (port->port_id == 2)
		trace_printk("Rx %d bytes\n", port->icount.rx - rx_count);
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
	struct mcf_platform_uart *platp = dev_get_platdata(&pdev->dev);
	struct uart_port *port;
	struct mcf_uart *pp;
	struct resource *res;
	void __iomem *base;
	int i, ret;

	for (i = 0; (i < MCF_MAXPORTS); i++) {
		port = &mcf_ports[i].port;

		port->line = i;
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
		
		base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
		if (IS_ERR(base)) {
			ret = PTR_ERR(base);
			dev_err(&pdev->dev, "Could not get resources: %d\n", ret);
		}
		port->membase = base;
		mcf_dma_setup(port);
		
		uart_add_one_port(&mcf_driver, port);
	}

	return 0;
}

/****************************************************************************/

static void mcf_remove(struct platform_device *pdev)
{
	struct uart_port *port;
	int i;

	for (i = 0; (i < MCF_MAXPORTS); i++) {
		port = &mcf_ports[i].port;
		if (port)
			uart_remove_one_port(&mcf_driver, port);
	}
}

/****************************************************************************/

static struct platform_driver mcf_platform_driver = {
	.probe		= mcf_probe,
	.remove_new	= mcf_remove,
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
