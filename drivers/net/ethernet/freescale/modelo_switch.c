/*
 *  L2 switch Controller (Etheren switch) driver for MCF5441x.
 *
 *  Copyright (C) 2010-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *    Shrek Wu (B16972@freescale.com)
 *    Alison Wang (b18965@freescale.com)
 *    Jason Jin (Jason.jin@freescale.com)
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/ptrace.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/phy.h>
#include <linux/kthread.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/signal.h>
#include <linux/wait.h>

#include <asm/irq.h>
#include <asm/pgtable.h>
#include <asm/cacheflush.h>
#include <asm/coldfire.h>
#include <asm/mcfsim.h>
#include "modelo_switch.h"

#define	SWITCH_MAX_PORTS	1


/* Interrupt events/masks.
*/
#define FEC_ENET_HBERR	((uint)0x80000000)	/* Heartbeat error */
#define FEC_ENET_BABR	((uint)0x40000000)	/* Babbling receiver */
#define FEC_ENET_BABT	((uint)0x20000000)	/* Babbling transmitter */
#define FEC_ENET_GRA	((uint)0x10000000)	/* Graceful stop complete */
#define FEC_ENET_TXF	((uint)0x08000000)	/* Full frame transmitted */
#define FEC_ENET_TXB	((uint)0x04000000)	/* A buffer was transmitted */
#define FEC_ENET_RXF	((uint)0x02000000)	/* Full frame received */
#define FEC_ENET_RXB	((uint)0x01000000)	/* A buffer was received */
#define FEC_ENET_MII	((uint)0x00800000)	/* MII interrupt */
#define FEC_ENET_EBERR	((uint)0x00400000)	/* SDMA bus error */

static int switch_enet_open(struct net_device *dev);
static int switch_enet_start_xmit(struct sk_buff *skb, struct net_device *dev);
static irqreturn_t switch_enet_interrupt(int irq, void *dev_id);
static void switch_enet_tx(struct net_device *dev);
static void switch_enet_rx(struct net_device *dev);
static int switch_enet_close(struct net_device *dev);
static void set_multicast_list(struct net_device *dev);
static void switch_restart(struct net_device *dev, int duplex);
static void switch_stop(struct net_device *dev);
static void switch_set_mac_address(struct net_device *dev);

#define		NMII	20

/* Make MII read/write commands for the FEC.
*/
#define mk_mii_read(REG)	(0x60020000 | ((REG & 0x1f) << 18))
#define mk_mii_write(REG, VAL)	(0x50020000 | ((REG & 0x1f) << 18) | \
						(VAL & 0xffff))

/* Transmitter timeout.
*/
#define TX_TIMEOUT (2*HZ)

/*last read entry from learning interface*/
eswPortInfo g_info;
/* switch ports status */
struct port_status ports_link_status;

/* the user space pid, used to send the link change to user space */
long user_pid = 1;

/* This could be done in a separate driver ? */
static u8 switch_enabled;
static struct platform_device *pdev_copy;
static struct delayed_work	config_workqueue;
#define SWE_POLL_TIMING	msecs_to_jiffies(1000)

/* ----------------------------------------------------------------*/
/*
 * Calculate Galois Field Arithmetic CRC for Polynom x^8+x^2+x+1.
 * It omits the final shift in of 8 zeroes a "normal" CRC would do
 * (getting the remainder).
 *
 *  Examples (hexadecimal values):<br>
 *   10-11-12-13-14-15  => CRC=0xc2
 *   10-11-cc-dd-ee-00  => CRC=0xe6
 *
 *   param: pmacaddress
 *          A 6-byte array with the MAC address.
 *          The first byte is the first byte transmitted
 *   return The 8-bit CRC in bits 7:0
 */
static int crc8_calc(unsigned char *pmacaddress)
{
	/* byte index */
	int byt;
	/* bit index */
	int bit;
	int inval;
	int crc;
	/* preset */
	crc   = 0x12;
	for (byt = 0; byt < 6; byt++) {
		inval = (((int)pmacaddress[byt]) & 0xff);
		/*
		 * shift bit 0 to bit 8 so all our bits
		 * travel through bit 8
		 * (simplifies below calc)
		 */
		inval <<= 8;

		for (bit = 0; bit < 8; bit++) {
			/* next input bit comes into d7 after shift */
			crc |= inval & 0x100;
			if (crc & 0x01)
				/* before shift  */
				crc ^= 0x1c0;

			crc >>= 1;
			inval >>= 1;
		}

	}
	/* upper bits are clean as we shifted in zeroes! */
	return crc;
}

static void read_atable(struct switch_enet_private *fep,
	int index, unsigned long *read_lo, unsigned long *read_hi)
{
	unsigned long atable_base = 0xFC0E0000;

	*read_lo = *((volatile unsigned long *)(atable_base + (index<<3)));
	*read_hi = *((volatile unsigned long *)(atable_base + (index<<3) + 4));
}

static void write_atable(struct switch_enet_private *fep,
	int index, unsigned long write_lo, unsigned long write_hi)
{
	unsigned long atable_base = 0xFC0E0000;

	*((volatile unsigned long *)(atable_base + (index<<3))) = write_lo;
	*((volatile unsigned long *)(atable_base + (index<<3) + 4)) = write_hi;
}

/* Read one element from the HW receive FIFO (Queue)
 * if available and return it.
 * return ms_HwPortInfo or null if no data is available
 */
static eswPortInfo *esw_portinfofifo_read(struct switch_enet_private *fep)
{
	volatile switch_t  *fecp;
	unsigned long tmp;

	fecp = fep->hwp;
	/* check learning record valid */
	if (fecp->ESW_LSR == 0)
		return NULL;

	/*read word from FIFO*/
	g_info.maclo = fecp->ESW_LREC0;

	/*but verify that we actually did so
	 * (0=no data available)*/
	if (g_info.maclo == 0)
		return NULL;

	/* read 2nd word from FIFO */
	tmp = fecp->ESW_LREC1;
	g_info.machi = tmp & 0xffff;
	g_info.hash  = (tmp >> 16) & 0xff;
	g_info.port  = (tmp >> 24) & 0xf;

	return &g_info;
}

/*
 * Clear complete MAC Look Up Table
 */
static void esw_clear_atable(struct switch_enet_private *fep)
{
	int index;
	for (index = 0; index < 2048; index++)
		write_atable(fep, index, 0, 0);
}

/*
 * pdates MAC address lookup table with a static entry
 * Searches if the MAC address is already there in the block and replaces
 * the older entry with new one. If MAC address is not there then puts a
 * new entry in the first empty slot available in the block
 *
 * mac_addr Pointer to the array containing MAC address to
 *          be put as static entry
 * port     Port bitmask numbers to be added in static entry,
 *          valid values are 1-7
 * priority Priority for the static entry in table
 *
 * return 0 for a successful update else -1  when no slot available
 */
static int esw_update_atable_static(unsigned char *mac_addr,
	unsigned int port, unsigned int priority,
	struct switch_enet_private *fep)
{
	unsigned long block_index, entry, index_end;
	unsigned long read_lo, read_hi;
	unsigned long write_lo, write_hi;

	write_lo = (unsigned long)((mac_addr[3] << 24) |
			(mac_addr[2] << 16) |
			(mac_addr[1] << 8) |
			mac_addr[0]);
	write_hi = (unsigned long)(0 |
			(port << AT_SENTRY_PORTMASK_shift) |
			(priority << AT_SENTRY_PRIO_shift) |
			(AT_ENTRY_TYPE_STATIC << AT_ENTRY_TYPE_shift) |
			(AT_ENTRY_RECORD_VALID << AT_ENTRY_VALID_shift) |
			(mac_addr[5] << 8) | (mac_addr[4]));

	block_index = GET_BLOCK_PTR(crc8_calc(mac_addr));
	index_end = block_index + ATABLE_ENTRY_PER_SLOT;
	/* Now search all the entries in the selected block */
	for (entry = block_index; entry < index_end; entry++) {
		read_atable(fep, entry, &read_lo, &read_hi);
		/*
		 * MAC address matched, so update the
		 * existing entry
		 * even if its a dynamic one
		 */
		if ((read_lo == write_lo) && ((read_hi & 0x0000ffff) ==
			 (write_hi & 0x0000ffff))) {
			write_atable(fep, entry, write_lo, write_hi);
			return 0;
		} else if (!(read_hi & (1 << 16))) {
			/*
			 * Fill this empty slot (valid bit zero),
			 * assuming no holes in the block
			 */
			write_atable(fep, entry, write_lo, write_hi);
			fep->atCurrEntries++;
			return 0;
		}
	}

	/* No space available for this static entry */
	return -1;
}

static int esw_update_atable_dynamic1(unsigned long write_lo, unsigned long write_hi,
		int block_index, unsigned int port, unsigned int currTime,
		struct switch_enet_private *fep)
{
	unsigned long entry, index_end;
	unsigned long read_lo, read_hi;
	unsigned long tmp;
	int time, timeold, indexold;

	/* prepare update port and timestamp */
	tmp = AT_ENTRY_RECORD_VALID << AT_ENTRY_VALID_shift;
	tmp |= AT_ENTRY_TYPE_DYNAMIC << AT_ENTRY_TYPE_shift;
	tmp |= currTime << AT_DENTRY_TIME_shift;
	tmp |= port << AT_DENTRY_PORT_shift;
	tmp |= write_hi;

	/*
	* linear search through all slot
	* entries and update if found
	*/
	index_end = block_index + ATABLE_ENTRY_PER_SLOT;
	/* Now search all the entries in the selected block */
	for (entry = block_index; entry < index_end; entry++) {
		read_atable(fep, entry, &read_lo, &read_hi);
		if ((read_lo == write_lo) &&
			((read_hi & 0x0000ffff) ==
			(write_hi & 0x0000ffff))) {
			/* found correct address,
			 * update timestamp. */
			write_atable(fep, entry, write_lo, tmp);
			return 0;
		} else if (!(read_hi & (1 << 16))) {
			/* slot is empty, then use it
			* for new entry
			* Note: There are no holes,
			* therefore cannot be any
			* more that need to be compared.
			*/
			write_atable(fep, entry, write_lo, tmp);
			/* statistics (we do it between writing
			*  .hi an .lo due to
			* hardware limitation...
			*/
			fep->atCurrEntries++;
			/* newly inserted */
			return 1;
		}
	}

	/*
	* no more entry available in block ...
	* overwrite oldest
	*/
	timeold = 0;
	indexold = 0;
	for (entry = block_index; entry < index_end; entry++) {
		read_atable(fep, entry, &read_lo, &read_hi);
		time = AT_EXTRACT_TIMESTAMP(read_hi);
		time = TIMEDELTA(currTime, time);
		if (time > timeold) {
			/* is it older ?*/
			timeold = time;
			indexold = entry;
		}
	}

	write_atable(fep, indexold, write_lo, tmp);
	/* Statistics (do it inbetween
	* writing to .lo and .hi*/
	fep->atBlockOverflows++;
	/* newly inserted */
	return 1;
}

static void esw_atable_dynamicms_del_entries_for_port(
	struct switch_enet_private *fep, int port_index)
{
	unsigned long read_lo, read_hi;
	unsigned int port_idx;
	int i;

	for (i = 0; i < ESW_ATABLE_MEM_NUM_ENTRIES; i++) {
		read_atable(fep, i, &read_lo, &read_hi);
		if (read_hi & (1 << 16)) {
			port_idx = AT_EXTRACT_PORT(read_hi);

			if (port_idx == port_index)
				write_atable(fep, i, 0, 0);
		}
	}
}

static void esw_atable_dynamicms_del_entries_for_other_port(
	struct switch_enet_private *fep,
	int port_index)
{
	unsigned long read_lo, read_hi;
	unsigned int port_idx;
	int i;

	for (i = 0; i < ESW_ATABLE_MEM_NUM_ENTRIES; i++) {
		read_atable(fep, i, &read_lo, &read_hi);
		if (read_hi & (1 << 16)) {
			port_idx = AT_EXTRACT_PORT(read_hi);

			if (port_idx != port_index)
				write_atable(fep, i, 0, 0);
		}
	}
}

/* dynamicms MAC address table learn and migration*/
static int esw_atable_dynamicms_learn_migration(
	struct switch_enet_private *fep,
	int currTime)
{
	eswPortInfo *pESWPortInfo;
	int index;
	int inserted = 0;

	pESWPortInfo = esw_portinfofifo_read(fep);
	/* Anything to learn */
	if (pESWPortInfo != 0) {
		/*get block index from lookup table*/
		index = GET_BLOCK_PTR(pESWPortInfo->hash);
		inserted = esw_update_atable_dynamic1(
			pESWPortInfo->maclo,
			pESWPortInfo->machi, index,
			pESWPortInfo->port, currTime, fep);
	}

	return 0;
}
/* -----------------------------------------------------------------*/
/*
 * esw_forced_forward
 * The frame is forwared to the forced destination ports.
 * It only replace the MAC lookup function,
 * all other filtering(eg.VLAN verification) act as normal
 */
static int esw_forced_forward(struct switch_enet_private *fep,
	int port1, int port2, int enable)
{
	unsigned long tmp = 0;
	volatile switch_t  *fecp;

	fecp = fep->hwp;

	/* Enable Forced forwarding for port num */
	if ((port1 == 1) && (port2 == 1))
		tmp |= MCF_ESW_P0FFEN_FD(3);
	else if (port1 == 1)
		/*Enable Forced forwarding for port 1 only*/
		tmp |= MCF_ESW_P0FFEN_FD(1);
	else if (port2 == 1)
		/*Enable Forced forwarding for port 2 only*/
		tmp |= MCF_ESW_P0FFEN_FD(2);
	else {
		printk(KERN_ERR "%s:do not support "
			"the forced forward mode"
			"port1 %x port2 %x\n",
			__func__, port1, port2);
		return -1;
	}

	if (enable == 1)
		tmp |= MCF_ESW_P0FFEN_FEN;
	else if (enable == 0)
		tmp &= ~MCF_ESW_P0FFEN_FEN;
	else {
		printk(KERN_ERR "%s: the enable %x is error\n",
			__func__, enable);
		return -2;
	}

	fecp->ESW_P0FFEN = tmp;
	return 0;
}

static void esw_get_forced_forward(
	struct switch_enet_private *fep,
	unsigned long *ulForceForward)
{
	volatile switch_t  *fecp;

	fecp = fep->hwp;
	*ulForceForward = fecp->ESW_P0FFEN;
}

static void esw_get_port_enable(
	struct switch_enet_private *fep,
	unsigned long *ulPortEnable)
{
	volatile switch_t  *fecp;

	fecp = fep->hwp;
	*ulPortEnable = fecp->ESW_PER;
}
/*
 * enable or disable port n tx or rx
 * tx_en 0 disable port n tx
 * tx_en 1 enable  port n tx
 * rx_en 0 disbale port n rx
 * rx_en 1 enable  port n rx
 */
static int esw_port_enable_config(struct switch_enet_private *fep,
	int port, int tx_en, int rx_en)
{
	unsigned long tmp = 0;
	volatile switch_t  *fecp;

	fecp = fep->hwp;
	tmp = fecp->ESW_PER;
	if (tx_en == 1) {
		if (port == 0)
			tmp |= MCF_ESW_PER_TE0;
		else if (port == 1)
			tmp |= MCF_ESW_PER_TE1;
		else if (port == 2)
			tmp |= MCF_ESW_PER_TE2;
		else {
			printk(KERN_ERR "%s:do not support the"
				" port %x tx enable\n",
				__func__, port);
			return -1;
		}
	} else if (tx_en == 0) {
		if (port == 0)
			tmp &= (~MCF_ESW_PER_TE0);
		else if (port == 1)
			tmp &= (~MCF_ESW_PER_TE1);
		else if (port == 2)
			tmp &= (~MCF_ESW_PER_TE2);
		else {
			printk(KERN_ERR "%s:do not support "
				"the port %x tx disable\n",
				__func__, port);
			return -2;
		}
	} else {
		printk(KERN_ERR "%s:do not support the port %x"
			" tx op value %x\n",
			__func__, port, tx_en);
		return -3;
	}

	if (rx_en == 1) {
		if (port == 0)
			tmp |= MCF_ESW_PER_RE0;
		else if (port == 1)
			tmp |= MCF_ESW_PER_RE1;
		else if (port == 2)
			tmp |= MCF_ESW_PER_RE2;
		else {
			printk(KERN_ERR "%s:do not support the "
				"port %x rx enable\n",
				__func__, port);
			return -4;
		}
	} else if (rx_en == 0) {
		if (port == 0)
			tmp &= (~MCF_ESW_PER_RE0);
		else if (port == 1)
			tmp &= (~MCF_ESW_PER_RE1);
		else if (port == 2)
			tmp &= (~MCF_ESW_PER_RE2);
		else {
			printk(KERN_ERR "%s:do not support the "
				"port %x rx disable\n",
				__func__, port);
			return -5;
		}
	} else {
		printk(KERN_ERR "%s:do not support the port %x"
			" rx op value %x\n",
			__func__, port, tx_en);
		return -6;
	}

	fecp->ESW_PER = tmp;
	return 0;
}


static void esw_get_port_broadcast(struct switch_enet_private *fep,
			unsigned long *ulPortBroadcast)
{
	volatile switch_t  *fecp;

	fecp = fep->hwp;
	*ulPortBroadcast = fecp->ESW_DBCR;
}

static int esw_port_broadcast_config(struct switch_enet_private *fep,
			int port, int enable)
{
	unsigned long tmp = 0;
	volatile switch_t  *fecp;

	fecp = fep->hwp;

	if ((port > 2) || (port < 0)) {
		printk(KERN_ERR "%s:do not support the port %x"
			" default broadcast\n",
			__func__, port);
		return -1;
	}

	tmp = fecp->ESW_DBCR;
	if (enable == 1) {
		if (port == 0)
			tmp |= MCF_ESW_DBCR_P0;
		else if (port == 1)
			tmp |= MCF_ESW_DBCR_P1;
		else if (port == 2)
			tmp |= MCF_ESW_DBCR_P2;
	} else if (enable == 0) {
		if (port == 0)
			tmp &= ~MCF_ESW_DBCR_P0;
		else if (port == 1)
			tmp &= ~MCF_ESW_DBCR_P1;
		else if (port == 2)
			tmp &= ~MCF_ESW_DBCR_P2;
	}

	fecp->ESW_DBCR = tmp;
	return 0;
}


static void esw_get_port_multicast(struct switch_enet_private *fep,
	unsigned long *ulPortMulticast)
{
	volatile switch_t  *fecp;

	fecp = fep->hwp;
	*ulPortMulticast = fecp->ESW_DMCR;
}

static int esw_port_multicast_config(struct switch_enet_private *fep,
	int port, int enable)
{
	unsigned long tmp = 0;
	volatile switch_t  *fecp;

	fecp = fep->hwp;

	if ((port > 2) || (port < 0)) {
		printk(KERN_ERR "%s:do not support the port %x"
			" default broadcast\n",
			__func__, port);
		return -1;
	}

	tmp = fecp->ESW_DMCR;
	if (enable == 1) {
		if (port == 0)
			tmp |= MCF_ESW_DMCR_P0;
		else if (port == 1)
			tmp |= MCF_ESW_DMCR_P1;
		else if (port == 2)
			tmp |= MCF_ESW_DMCR_P2;
	} else if (enable == 0) {
		if (port == 0)
			tmp &= ~MCF_ESW_DMCR_P0;
		else if (port == 1)
			tmp &= ~MCF_ESW_DMCR_P1;
		else if (port == 2)
			tmp &= ~MCF_ESW_DMCR_P2;
	}

	fecp->ESW_DMCR = tmp;
	return 0;
}


static void esw_get_port_blocking(struct switch_enet_private *fep,
	unsigned long *ulPortBlocking)
{
	volatile switch_t  *fecp;

	fecp = fep->hwp;
	*ulPortBlocking = (fecp->ESW_BKLR & 0x0000000f);
}

static int esw_port_blocking_config(struct switch_enet_private *fep,
	int port, int enable)
{
	unsigned long tmp = 0;
	volatile switch_t  *fecp;

	fecp = fep->hwp;

	if ((port > 2) || (port < 0)) {
		printk(KERN_ERR "%s:do not support the port %x"
			" default broadcast\n",
			__func__, port);
		return -1;
	}

	tmp = fecp->ESW_BKLR;
	if (enable == 1) {
		if (port == 0)
			tmp |= MCF_ESW_BKLR_BE0;
		else if (port == 1)
			tmp |= MCF_ESW_BKLR_BE1;
		else if (port == 2)
			tmp |= MCF_ESW_BKLR_BE2;
	} else if (enable == 0) {
		if (port == 0)
			tmp &= ~MCF_ESW_BKLR_BE0;
		else if (port == 1)
			tmp &= ~MCF_ESW_BKLR_BE1;
		else if (port == 2)
			tmp &= ~MCF_ESW_BKLR_BE2;
	}

	fecp->ESW_BKLR = tmp;
	return 0;
}


static void esw_get_port_learning(struct switch_enet_private *fep,
	unsigned long *ulPortLearning)
{
	volatile switch_t  *fecp;

	fecp = fep->hwp;
	*ulPortLearning = (fecp->ESW_BKLR & 0x000f0000) >> 16;
}

static int esw_port_learning_config(struct switch_enet_private *fep,
	int port, int disable)
{
	unsigned long tmp = 0;
	volatile switch_t  *fecp;

	fecp = fep->hwp;

	if ((port > 2) || (port < 0)) {
		printk(KERN_ERR "%s:do not support the port %x"
			" default broadcast\n",
			__func__, port);
		return -1;
	}

	tmp = fecp->ESW_BKLR;
	if (disable == 0) {
		fep->learning_irqhandle_enable = 0;
		if (port == 0)
			tmp |= MCF_ESW_BKLR_LD0;
		else if (port == 1)
			tmp |= MCF_ESW_BKLR_LD1;
		else if (port == 2)
			tmp |= MCF_ESW_BKLR_LD2;
	} else if (disable == 1) {
		if (port == 0)
			tmp &= ~MCF_ESW_BKLR_LD0;
		else if (port == 1)
			tmp &= ~MCF_ESW_BKLR_LD1;
		else if (port == 2)
			tmp &= ~MCF_ESW_BKLR_LD2;
	}

	fecp->ESW_BKLR = tmp;
	return 0;
}
/*********************************************************************/
/*
 * Checks IP Snoop options of handling the snooped frame.
 * mode 0 : The snooped frame is forward only to management port
 * mode 1 : The snooped frame is copy to management port and
 *              normal forwarding is checked.
 * mode 2 : The snooped frame is discarded.
 * mode 3 : Disable the ip snoop function
 * ip_header_protocol : the IP header protocol field
 */
static int esw_ip_snoop_config(struct switch_enet_private *fep,
		int mode, unsigned long ip_header_protocol)
{
	volatile switch_t  *fecp;
	unsigned long tmp = 0, protocol_type = 0;
	int num = 0;

	fecp = fep->hwp;
	/* Config IP Snooping */
	if (mode == 0) {
		/* Enable IP Snooping */
		tmp = MCF_ESW_IPSNP_EN;
		tmp |= MCF_ESW_IPSNP_MODE(0);/*For Forward*/
	} else if (mode == 1) {
		/* Enable IP Snooping */
		tmp = MCF_ESW_IPSNP_EN;
		/*For Forward and copy_to_mangmnt_port*/
		tmp |= MCF_ESW_IPSNP_MODE(1);
	} else if (mode == 2) {
		/* Enable IP Snooping */
		tmp = MCF_ESW_IPSNP_EN;
		tmp |= MCF_ESW_IPSNP_MODE(2);/*discard*/
	} else if (mode == 3) {
		/* disable IP Snooping */
		tmp = MCF_ESW_IPSNP_EN;
		tmp &= ~MCF_ESW_IPSNP_EN;
	} else {
		printk(KERN_ERR "%s: the mode %x "
			"we do not support\n", __func__, mode);
		return -1;
	}

	protocol_type = ip_header_protocol;
	for (num = 0; num < 8; num++) {
		if (protocol_type ==
				AT_EXTRACT_IP_PROTOCOL(fecp->ESW_IPSNP[num])) {
			fecp->ESW_IPSNP[num] =
				tmp | MCF_ESW_IPSNP_PROTOCOL(protocol_type);
			break;
		} else if (!(fecp->ESW_IPSNP[num])) {
			fecp->ESW_IPSNP[num] =
				tmp | MCF_ESW_IPSNP_PROTOCOL(protocol_type);
			break;
		}
	}
	if (num == 8) {
		printk(KERN_INFO "IP snooping table is full\n");
		return 0;
	}

	return 0;
}

static void esw_get_ip_snoop_config(struct switch_enet_private *fep,
	unsigned long *ulpESW_IPSNP)
{
	int i;
	volatile switch_t  *fecp;

	fecp = fep->hwp;
	for (i = 0; i < 8; i++)
		*(ulpESW_IPSNP + i) = fecp->ESW_IPSNP[i];
}
/*
 * Checks TCP/UDP Port Snoop options of handling the snooped frame.
 * mode 0 : The snooped frame is forward only to management port
 * mode 1 : The snooped frame is copy to management port and
 *              normal forwarding is checked.
 * mode 2 : The snooped frame is discarded.
 * mode 3 : Disable the TCP/UDP port snoop function
 * compare_port : port number in the TCP/UDP header
 * compare_num 1: TCP/UDP source port number is compared
 * compare_num 2: TCP/UDP destination port number is compared
 * compare_num 3: TCP/UDP source and destination port number is compared
 */
static int esw_tcpudp_port_snoop_config(struct switch_enet_private *fep,
		int mode, int compare_port, int compare_num)
{
	volatile switch_t  *fecp;
	unsigned long tmp;
	int num;

	fecp = fep->hwp;

	/* Enable TCP/UDP port Snooping */
	tmp = MCF_ESW_PSNP_EN;
	if (mode == 0)
		tmp |= MCF_ESW_PSNP_MODE(0);/*For Forward*/
	else if (mode == 1)/*For Forward and copy_to_mangmnt_port*/
		tmp |= MCF_ESW_PSNP_MODE(1);
	else if (mode == 2)
		tmp |= MCF_ESW_PSNP_MODE(2);/*discard*/
	else if (mode == 3) /*disable the port function*/
		tmp &= (~MCF_ESW_PSNP_EN);
	else {
		printk(KERN_ERR "%s: the mode %x we do not support\n",
			__func__, mode);
		return -1;
	}

	if (compare_num == 1)
		tmp |= MCF_ESW_PSNP_CS;
	else if (compare_num == 2)
		tmp |= MCF_ESW_PSNP_CD;
	else if (compare_num == 3)
		tmp |= MCF_ESW_PSNP_CD | MCF_ESW_PSNP_CS;
	else {
		printk(KERN_ERR "%s: the compare port address %x"
			" we do not support\n",
			__func__, compare_num);
		return -1;
	}

	for (num = 0; num < 8; num++) {
		if (compare_port ==
				AT_EXTRACT_TCP_UDP_PORT(fecp->ESW_PSNP[num])) {
			fecp->ESW_PSNP[num] =
				tmp | MCF_ESW_PSNP_PORT_COMPARE(compare_port);
			break;
		} else if (!(fecp->ESW_PSNP[num])) {
			fecp->ESW_PSNP[num] =
				tmp | MCF_ESW_PSNP_PORT_COMPARE(compare_port);
			break;
		}
	}
	if (num == 8) {
		printk(KERN_INFO "TCP/UDP port snooping table is full\n");
		return 0;
	}

	return 0;
}

static void esw_get_tcpudp_port_snoop_config(
	struct switch_enet_private *fep,
	unsigned long *ulpESW_PSNP)
{
	int i;
	volatile switch_t  *fecp;

	fecp = fep->hwp;
	for (i = 0; i < 8; i++)
		*(ulpESW_PSNP + i) = fecp->ESW_PSNP[i];
}
/*-----------------mirror----------------------------------------*/
static void esw_get_port_mirroring(struct switch_enet_private *fep)
{
	volatile switch_t  *fecp;

	fecp = fep->hwp;

	printk(KERN_INFO "Mirror Port: %1ld   Egress Port Match:%s    "
		"Ingress Port Match:%s\n", fecp->ESW_MCR & 0xf,
		(fecp->ESW_MCR >> 6) & 1 ? "Y" : "N",
		(fecp->ESW_MCR >> 5) & 1 ? "Y" : "N");

	if ((fecp->ESW_MCR >> 6) & 1)
		printk(KERN_INFO "Egress Port to be mirrored: Port %ld\n",
			fecp->ESW_EGMAP >> 1);
	if ((fecp->ESW_MCR >> 5) & 1)
		printk(KERN_INFO "Ingress Port to be mirrored: Port %ld\n",
			fecp->ESW_INGMAP >> 1);

	printk(KERN_INFO "Egress Des Address Match:%s    "
		"Egress Src Address Match:%s\n",
		(fecp->ESW_MCR >> 10) & 1 ? "Y" : "N",
		(fecp->ESW_MCR >> 9) & 1 ? "Y" : "N");
	printk(KERN_INFO "Ingress Des Address Match:%s   "
		"Ingress Src Address Match:%s\n",
		(fecp->ESW_MCR >> 8) & 1 ? "Y" : "N",
		(fecp->ESW_MCR >> 7) & 1 ? "Y" : "N");

	if ((fecp->ESW_MCR >> 10) & 1)
		printk(KERN_INFO "Egress Des Address to be mirrored: "
			"%02lx-%02lx-%02lx-%02lx-%02lx-%02lx\n",
			fecp->ESW_ENGDAL & 0xff, (fecp->ESW_ENGDAL >> 8) & 0xff,
			(fecp->ESW_ENGDAL >> 16) & 0xff,
			(fecp->ESW_ENGDAL >> 24) & 0xff,
			fecp->ESW_ENGDAH & 0xff,
			(fecp->ESW_ENGDAH >> 8) & 0xff);
	if ((fecp->ESW_MCR >> 9) & 1)
		printk("Egress Src Address to be mirrored: "
			"%02lx-%02lx-%02lx-%02lx-%02lx-%02lx\n",
			fecp->ESW_ENGSAL & 0xff, (fecp->ESW_ENGSAL >> 8) & 0xff,
			(fecp->ESW_ENGSAL >> 16) & 0xff,
			(fecp->ESW_ENGSAL >> 24) & 0xff,
			fecp->ESW_ENGSAH & 0xff,
			(fecp->ESW_ENGSAH >> 8) & 0xff);
	if ((fecp->ESW_MCR >> 8) & 1)
		printk("Ingress Des Address to be mirrored: "
			"%02lx-%02lx-%02lx-%02lx-%02lx-%02lx\n",
			fecp->ESW_INGDAL & 0xff, (fecp->ESW_INGDAL >> 8) & 0xff,
			(fecp->ESW_INGDAL >> 16) & 0xff,
			(fecp->ESW_INGDAL >> 24) & 0xff,
			fecp->ESW_INGDAH & 0xff,
			(fecp->ESW_INGDAH >> 8) & 0xff);
	if ((fecp->ESW_MCR >> 7) & 1)
		printk("Ingress Src Address to be mirrored: "
			"%02lx-%02lx-%02lx-%02lx-%02lx-%02lx\n",
			fecp->ESW_INGSAL & 0xff, (fecp->ESW_INGSAL >> 8) & 0xff,
			(fecp->ESW_INGSAL >> 16) & 0xff,
			(fecp->ESW_INGSAL >> 24) & 0xff,
			fecp->ESW_INGSAH & 0xff,
			(fecp->ESW_INGSAH >> 8) & 0xff);
}

static int esw_port_mirroring_config_port_match(struct switch_enet_private *fep,
	int mirror_port, int port_match_en, int port)
{
	volatile switch_t  *fecp;
	unsigned long tmp = 0;

	fecp = fep->hwp;

	tmp = fecp->ESW_MCR;
	if (mirror_port != (tmp & 0xf))
		tmp = 0;

	switch (port_match_en) {
	case MIRROR_EGRESS_PORT_MATCH:
		tmp |= MCF_ESW_MCR_EGMAP;
		if (port == 0)
			fecp->ESW_EGMAP = MCF_ESW_EGMAP_EG0;
		else if (port == 1)
			fecp->ESW_EGMAP = MCF_ESW_EGMAP_EG1;
		else if (port == 2)
			fecp->ESW_EGMAP = MCF_ESW_EGMAP_EG2;
		break;
	case MIRROR_INGRESS_PORT_MATCH:
		tmp |= MCF_ESW_MCR_INGMAP;
		if (port == 0)
			fecp->ESW_INGMAP = MCF_ESW_INGMAP_ING0;
		else if (port == 1)
			fecp->ESW_INGMAP = MCF_ESW_INGMAP_ING1;
		else if (port == 2)
			fecp->ESW_INGMAP = MCF_ESW_INGMAP_ING2;
		break;
	default:
		tmp = 0;
		break;
	}

	tmp = tmp & 0x07e0;
	if (port_match_en)
		tmp |= MCF_ESW_MCR_MEN | MCF_ESW_MCR_PORT(mirror_port);

	fecp->ESW_MCR = tmp;
	return 0;
}

static int esw_port_mirroring_config(struct switch_enet_private *fep,
	int mirror_port, int port, int mirror_enable,
	unsigned char *src_mac, unsigned char *des_mac,
	int egress_en, int ingress_en,
	int egress_mac_src_en, int egress_mac_des_en,
	int ingress_mac_src_en, int ingress_mac_des_en)
{
	volatile switch_t  *fecp;
	unsigned long tmp;

	fecp = fep->hwp;

	/*mirroring config*/
	tmp = 0;
	if (egress_en == 1) {
		tmp |= MCF_ESW_MCR_EGMAP;
		if (port == 0)
			fecp->ESW_EGMAP = MCF_ESW_EGMAP_EG0;
		else if (port == 1)
			fecp->ESW_EGMAP = MCF_ESW_EGMAP_EG1;
		else if (port == 2)
			fecp->ESW_EGMAP = MCF_ESW_EGMAP_EG2;
		else {
			printk(KERN_ERR "%s: the port %x we do not support\n",
					__func__, port);
			return -1;
		}
	} else if (egress_en == 0) {
		tmp &= (~MCF_ESW_MCR_EGMAP);
	} else {
		printk(KERN_ERR "%s: egress_en %x we do not support\n",
			__func__, egress_en);
		return -1;
	}

	if (ingress_en == 1) {
		tmp |= MCF_ESW_MCR_INGMAP;
		if (port == 0)
			fecp->ESW_INGMAP = MCF_ESW_INGMAP_ING0;
		else if (port == 1)
			fecp->ESW_INGMAP = MCF_ESW_INGMAP_ING1;
		else if (port == 2)
			fecp->ESW_INGMAP = MCF_ESW_INGMAP_ING2;
		else {
			printk(KERN_ERR "%s: the port %x we do not support\n",
				__func__, port);
			return -1;
		}
	} else if (ingress_en == 0) {
		tmp &= ~MCF_ESW_MCR_INGMAP;
	} else{
		printk(KERN_ERR "%s: ingress_en %x we do not support\n",
				__func__, ingress_en);
		return -1;
	}

	if (egress_mac_src_en == 1) {
		tmp |= MCF_ESW_MCR_EGSA;
		fecp->ESW_ENGSAH = (src_mac[5] << 8) | (src_mac[4]);
		fecp->ESW_ENGSAL = (unsigned long)((src_mac[3] << 24) |
					(src_mac[2] << 16) |
					(src_mac[1] << 8) |
					src_mac[0]);
	} else if (egress_mac_src_en == 0) {
		tmp &= ~MCF_ESW_MCR_EGSA;
	} else {
		printk(KERN_ERR "%s: egress_mac_src_en  %x we do not support\n",
			__func__, egress_mac_src_en);
		return -1;
	}

	if (egress_mac_des_en == 1) {
		tmp |= MCF_ESW_MCR_EGDA;
		fecp->ESW_ENGDAH = (des_mac[5] << 8) | (des_mac[4]);
		fecp->ESW_ENGDAL = (unsigned long)((des_mac[3] << 24) |
					(des_mac[2] << 16) |
					(des_mac[1] << 8) |
					des_mac[0]);
	} else if (egress_mac_des_en == 0) {
		tmp &= ~MCF_ESW_MCR_EGDA;
	} else {
		printk(KERN_ERR "%s: egress_mac_des_en  %x we do not support\n",
			__func__, egress_mac_des_en);
		return -1;
	}

	if (ingress_mac_src_en == 1) {
		tmp |= MCF_ESW_MCR_INGSA;
		fecp->ESW_INGSAH = (src_mac[5] << 8) | (src_mac[4]);
		fecp->ESW_INGSAL = (unsigned long)((src_mac[3] << 24) |
					(src_mac[2] << 16) |
					(src_mac[1] << 8) |
					src_mac[0]);
	} else if (ingress_mac_src_en == 0) {
		tmp &= ~MCF_ESW_MCR_INGSA;
	} else {
		printk(KERN_ERR "%s: ingress_mac_src_en  %x we do not support\n",
			__func__, ingress_mac_src_en);
		return -1;
	}

	if (ingress_mac_des_en == 1) {
		tmp |= MCF_ESW_MCR_INGDA;
		fecp->ESW_INGDAH = (des_mac[5] << 8) | (des_mac[4]);
		fecp->ESW_INGDAL = (unsigned long)((des_mac[3] << 24) |
					(des_mac[2] << 16) |
					(des_mac[1] << 8) |
					des_mac[0]);
	} else if (ingress_mac_des_en == 0) {
		tmp &= ~MCF_ESW_MCR_INGDA;
	} else {
		printk(KERN_ERR "%s: ingress_mac_des_en  %x we do not support\n",
			__func__, ingress_mac_des_en);
		return -1;
	}

	if (mirror_enable == 1)
		tmp |= MCF_ESW_MCR_MEN | MCF_ESW_MCR_PORT(mirror_port);
	else if (mirror_enable == 0)
		tmp &= ~MCF_ESW_MCR_MEN;
	else
		printk(KERN_ERR "%s: the mirror enable %x is error\n",
			__func__, mirror_enable);


	fecp->ESW_MCR = tmp;
	return 0;
}

static int esw_port_mirroring_config_addr_match(struct switch_enet_private *fep,
	int mirror_port, int addr_match_enable, unsigned char *mac_addr)
{
	volatile switch_t  *fecp;
	unsigned long tmp = 0;

	fecp = fep->hwp;

	tmp = fecp->ESW_MCR;
	if (mirror_port != (tmp & 0xf))
		tmp = 0;

	switch (addr_match_enable) {
	case MIRROR_EGRESS_SOURCE_MATCH:
		tmp |= MCF_ESW_MCR_EGSA;
		fecp->ESW_ENGSAH = (mac_addr[5] << 8) | (mac_addr[4]);
		fecp->ESW_ENGSAL = (unsigned long)((mac_addr[3] << 24) |
			(mac_addr[2] << 16) | (mac_addr[1] << 8) | mac_addr[0]);
		break;
	case MIRROR_INGRESS_SOURCE_MATCH:
		tmp |= MCF_ESW_MCR_INGSA;
		fecp->ESW_INGSAH = (mac_addr[5] << 8) | (mac_addr[4]);
		fecp->ESW_INGSAL = (unsigned long)((mac_addr[3] << 24) |
			(mac_addr[2] << 16) | (mac_addr[1] << 8) | mac_addr[0]);
		break;
	case MIRROR_EGRESS_DESTINATION_MATCH:
		tmp |= MCF_ESW_MCR_EGDA;
		fecp->ESW_ENGDAH = (mac_addr[5] << 8) | (mac_addr[4]);
		fecp->ESW_ENGDAL = (unsigned long)((mac_addr[3] << 24) |
			(mac_addr[2] << 16) | (mac_addr[1] << 8) | mac_addr[0]);
		break;
	case MIRROR_INGRESS_DESTINATION_MATCH:
		tmp |= MCF_ESW_MCR_INGDA;
		fecp->ESW_INGDAH = (mac_addr[5] << 8) | (mac_addr[4]);
		fecp->ESW_INGDAL = (unsigned long)((mac_addr[3] << 24) |
			(mac_addr[2] << 16) | (mac_addr[1] << 8) | mac_addr[0]);
		break;
	default:
		tmp = 0;
		break;
	}

	tmp = tmp & 0x07e0;
	if (addr_match_enable)
		tmp |= MCF_ESW_MCR_MEN | MCF_ESW_MCR_PORT(mirror_port);

	fecp->ESW_MCR = tmp;
	return 0;
}

static void esw_get_vlan_verification(struct switch_enet_private *fep,
	unsigned long *ulValue)
{
	volatile switch_t  *fecp;
	fecp = fep->hwp;
	*ulValue = fecp->ESW_VLANV;
}

static int esw_set_vlan_verification(struct switch_enet_private *fep, int port,
	int vlan_domain_verify_en, int vlan_discard_unknown_en)
{
	volatile switch_t  *fecp;

	fecp = fep->hwp;
	if ((port < 0) || (port > 2)) {
		printk(KERN_ERR "%s: do not support the port %d\n",
			__func__, port);
		return -1;
	}

	if (vlan_domain_verify_en == 1) {
		if (port == 0)
			fecp->ESW_VLANV |= MCF_ESW_VLANV_VV0;
		else if (port == 1)
			fecp->ESW_VLANV |= MCF_ESW_VLANV_VV1;
		else if (port == 2)
			fecp->ESW_VLANV |= MCF_ESW_VLANV_VV2;
	} else if (vlan_domain_verify_en == 0) {
		if (port == 0)
			fecp->ESW_VLANV &= ~MCF_ESW_VLANV_VV0;
		else if (port == 1)
			fecp->ESW_VLANV &= ~MCF_ESW_VLANV_VV1;
		else if (port == 2)
			fecp->ESW_VLANV &= ~MCF_ESW_VLANV_VV2;
	} else {
		printk(KERN_INFO "%s: donot support "
			"vlan_domain_verify %x\n",
			__func__, vlan_domain_verify_en);
		return -2;
	}

	if (vlan_discard_unknown_en == 1) {
		if (port == 0)
			fecp->ESW_VLANV |= MCF_ESW_VLANV_DU0;
		else if (port == 1)
			fecp->ESW_VLANV |= MCF_ESW_VLANV_DU1;
		else if (port == 2)
			fecp->ESW_VLANV |= MCF_ESW_VLANV_DU2;
	} else if (vlan_discard_unknown_en == 0) {
		if (port == 0)
			fecp->ESW_VLANV &= ~MCF_ESW_VLANV_DU0;
		else if (port == 1)
			fecp->ESW_VLANV &= ~MCF_ESW_VLANV_DU1;
		else if (port == 2)
			fecp->ESW_VLANV &= ~MCF_ESW_VLANV_DU2;
	} else {
		printk(KERN_INFO "%s: donot support "
			"vlan_discard_unknown %x\n",
			__func__, vlan_discard_unknown_en);
		return -3;
	}

	return 0;
}

static void esw_get_vlan_resolution_table(struct switch_enet_private *fep,
	struct eswVlanTableItem *tableaddr)
{
	volatile switch_t  *fecp;
	int vnum = 0;
	int i;

	fecp = fep->hwp;
	for (i = 0; i < 32; i++) {
		if (fecp->ESW_VRES[i]) {
			tableaddr->table[i].port_vlanid =
				fecp->ESW_VRES[i] >> 3;
			tableaddr->table[i].vlan_domain_port =
				fecp->ESW_VRES[i] & 7;
			vnum++;
		}
	}
	tableaddr->valid_num = vnum;
}

static int esw_set_vlan_id(struct switch_enet_private *fep, unsigned long configData)
{
	volatile switch_t  *fecp;
	int i;

	fecp = fep->hwp;

	for (i = 0; i < 32; i++) {
		if (fecp->ESW_VRES[i] == 0) {
			fecp->ESW_VRES[i] = MCF_ESW_VRES_VLANID(configData);
			return 0;
		} else if (((fecp->ESW_VRES[i] >> 3) & 0xfff) == configData) {
			printk(KERN_INFO "The VLAN already exists\n");
			return 0;
		}
	}

	printk(KERN_INFO "The VLAN can't create, because VLAN table is full\n");
	return 0;
}

static int esw_set_vlan_id_cleared(struct switch_enet_private *fep,
		unsigned long configData)
{
	volatile switch_t  *fecp;
	int i;

	fecp = fep->hwp;

	for (i = 0; i < 32; i++) {
		if (((fecp->ESW_VRES[i] >> 3) & 0xfff) == configData) {
			fecp->ESW_VRES[i] = 0;
			break;
		}
	}
	return 0;
}

static int esw_set_port_in_vlan_id(struct switch_enet_private *fep,
	       eswIoctlVlanResoultionTable configData)
{
	volatile switch_t  *fecp;
	int i;
	int lastnum = 0;

	fecp = fep->hwp;

	for (i = 0; i < 32; i++) {
		if (fecp->ESW_VRES[i] == 0) {
			lastnum = i;
			break;
		} else if (((fecp->ESW_VRES[i] >> 3) & 0xfff) ==
				configData.port_vlanid) {
			/* update the port members of this vlan */
			fecp->ESW_VRES[i] |= 1 << configData.vlan_domain_port;
			return 0;
		}
	}
	/* creat a new vlan in vlan table */
	fecp->ESW_VRES[lastnum] = MCF_ESW_VRES_VLANID(configData.port_vlanid) |
		(1 << configData.vlan_domain_port);
	return 0;
}

static int esw_set_vlan_resolution_table(struct switch_enet_private *fep,
	unsigned short port_vlanid, int vlan_domain_num,
	int vlan_domain_port)
{
	volatile switch_t  *fecp;

	fecp = fep->hwp;
	if ((vlan_domain_num < 0)
		|| (vlan_domain_num > 31)) {
		printk(KERN_ERR "%s: do not support the "
			"vlan_domain_num %d\n",
		__func__, vlan_domain_num);
		return -1;
	}

	if ((vlan_domain_port < 0)
		|| (vlan_domain_port > 7)) {
		printk(KERN_ERR "%s: do not support the "
			"vlan_domain_port %d\n",
			__func__, vlan_domain_port);
		return -2;
	}

	fecp->ESW_VRES[vlan_domain_num] =
		MCF_ESW_VRES_VLANID(port_vlanid)
		| vlan_domain_port;

	return 0;
}

static void esw_get_vlan_input_config(struct switch_enet_private *fep,
	eswIoctlVlanInputStatus *pVlanInputConfig)
{
	volatile switch_t  *fecp;
	int i;

	fecp = fep->hwp;
	for (i = 0; i < 3; i++)
		pVlanInputConfig->ESW_PID[i] = fecp->ESW_PID[i];

	pVlanInputConfig->ESW_VLANV  = fecp->ESW_VLANV;
	pVlanInputConfig->ESW_VIMSEL = fecp->ESW_VIMSEL;
	pVlanInputConfig->ESW_VIMEN  = fecp->ESW_VIMEN;

	for (i = 0; i < 32; i++)
		pVlanInputConfig->ESW_VRES[i] = fecp->ESW_VRES[i];
}


static int esw_vlan_input_process(struct switch_enet_private *fep,
	int port, int mode, unsigned short port_vlanid)
{
	volatile switch_t  *fecp;

	fecp = fep->hwp;

	if ((mode < 0) || (mode > 5)) {
		printk(KERN_ERR "%s: do not support the"
			" VLAN input processing mode %d\n",
			__func__, mode);
		return -1;
	}

	if ((port < 0) || (port > 3)) {
		printk(KERN_ERR "%s: do not support the port %d\n",
			__func__, mode);
		return -2;
	}

	fecp->ESW_PID[port] = MCF_ESW_PID_VLANID(port_vlanid);
	if (port == 0) {
		if (mode == 4)
			fecp->ESW_VIMEN &= ~MCF_ESW_VIMEN_EN0;
		else
			fecp->ESW_VIMEN |= MCF_ESW_VIMEN_EN0;

		fecp->ESW_VIMSEL &= ~MCF_ESW_VIMSEL_IM0(3);
		fecp->ESW_VIMSEL |= MCF_ESW_VIMSEL_IM0(mode);
	} else if (port == 1) {
		if (mode == 4)
			fecp->ESW_VIMEN &= ~MCF_ESW_VIMEN_EN1;
		else
			fecp->ESW_VIMEN |= MCF_ESW_VIMEN_EN1;

		fecp->ESW_VIMSEL &= ~MCF_ESW_VIMSEL_IM1(3);
		fecp->ESW_VIMSEL |= MCF_ESW_VIMSEL_IM1(mode);
	} else if (port == 2) {
		if (mode == 4)
			fecp->ESW_VIMEN &= ~MCF_ESW_VIMEN_EN2;
		else
			fecp->ESW_VIMEN |= MCF_ESW_VIMEN_EN2;

		fecp->ESW_VIMSEL &= ~MCF_ESW_VIMSEL_IM2(3);
		fecp->ESW_VIMSEL |= MCF_ESW_VIMSEL_IM2(mode);
	} else {
		printk(KERN_ERR "%s: do not support the port %d\n",
			__func__, port);
		return -2;
	}

	return 0;
}

static void esw_get_vlan_output_config(struct switch_enet_private *fep,
	unsigned long *ulVlanOutputConfig)
{
	volatile switch_t  *fecp;

	fecp = fep->hwp;
	*ulVlanOutputConfig = fecp->ESW_VOMSEL;
}

static int esw_vlan_output_process(struct switch_enet_private *fep,
	int port, int mode)
{
	volatile switch_t  *fecp;

	fecp = fep->hwp;

	if ((port < 0) || (port > 2)) {
		printk(KERN_ERR "%s: do not support the port %d\n",
			__func__, mode);
		return -1;
	}

	if (port == 0) {
		fecp->ESW_VOMSEL &= ~MCF_ESW_VOMSEL_OM0(3);
		fecp->ESW_VOMSEL |= MCF_ESW_VOMSEL_OM0(mode);
	} else if (port == 1) {
		fecp->ESW_VOMSEL &= ~MCF_ESW_VOMSEL_OM1(3);
		fecp->ESW_VOMSEL |= MCF_ESW_VOMSEL_OM1(mode);
	} else if (port == 2) {
		fecp->ESW_VOMSEL &= ~MCF_ESW_VOMSEL_OM2(3);
		fecp->ESW_VOMSEL |= MCF_ESW_VOMSEL_OM2(mode);
	} else {
		printk(KERN_ERR "%s: do not support the port %d\n",
			__func__, port);
		return -1;
	}

	return 0;
}

/*------------frame calssify and priority resolution------------*/
/*vlan priority lookup*/
static int esw_framecalssify_vlan_priority_lookup(struct switch_enet_private *fep,
	int port, int func_enable, int vlan_pri_table_num,
	int vlan_pri_table_value)
{
	volatile switch_t  *fecp;

	fecp = fep->hwp;

	if ((port < 0) || (port > 3)) {
		printk(KERN_ERR "%s: do not support the port %d\n",
			__func__, port);
		return -1;
	}

	if (func_enable == 0) {
		fecp->ESW_PRES[port] &= ~MCF_ESW_PRES_VLAN;
		printk(KERN_ERR "%s: disable port %d VLAN priority "
			"lookup function\n", __func__, port);
		return 0;
	}

	if ((vlan_pri_table_num < 0) || (vlan_pri_table_num > 7)) {
		printk(KERN_ERR "%s: do not support the priority %d\n",
			__func__, vlan_pri_table_num);
		return -1;
	}

	fecp->ESW_PVRES[port] |= ((vlan_pri_table_value & 0x3)
		<< (vlan_pri_table_num*3));
	/* enable port  VLAN priority lookup function*/
	fecp->ESW_PRES[port] |= MCF_ESW_PRES_VLAN;
	return 0;
}

static int esw_framecalssify_ip_priority_lookup(struct switch_enet_private *fep,
	int port, int func_enable, int ipv4_en, int ip_priority_num,
	int ip_priority_value)
{
	volatile switch_t  *fecp;
	unsigned long tmp = 0, tmp_prio = 0;

	fecp = fep->hwp;

	if ((port < 0) || (port > 3)) {
		printk(KERN_ERR "%s: do not support the port %d\n",
			__func__, port);
		return -1;
	}

	if (func_enable == 0) {
		fecp->ESW_PRES[port] &= ~MCF_ESW_PRES_IP;
		printk(KERN_ERR "%s: disable port %d ip priority "
			"lookup function\n", __func__, port);
		return 0;
	}

	/* IPV4 priority 64 entry table lookup*/
	/* IPv4 head 6 bit TOS field*/
	if (ipv4_en == 1) {
		if ((ip_priority_num < 0) || (ip_priority_num > 63)) {
			printk(KERN_ERR "%s: do not support the table entry %d\n",
				__func__, ip_priority_num);
			return -2;
		}
	} else { /* IPV6 priority 256 entry table lookup*/
		/* IPv6 head 8 bit COS field*/
		if ((ip_priority_num < 0) || (ip_priority_num > 255)) {
			printk(KERN_ERR "%s: do not support the table entry %d\n",
				__func__, ip_priority_num);
			return -3;
		}
	}

	/* IP priority  table lookup : address*/
	tmp = MCF_ESW_IPRES_ADDRESS(ip_priority_num);
	/* IP priority  table lookup : ipv4sel*/
	if (ipv4_en == 1)
		tmp = tmp | MCF_ESW_IPRES_IPV4SEL;
	/* IP priority  table lookup : priority*/
	if (port == 0)
		tmp |= MCF_ESW_IPRES_PRI0(ip_priority_value);
	else if (port == 1)
		tmp |= MCF_ESW_IPRES_PRI1(ip_priority_value);
	else if (port == 2)
		tmp |= MCF_ESW_IPRES_PRI2(ip_priority_value);

	/* configure*/
	fecp->ESW_IPRES = MCF_ESW_IPRES_READ |
		MCF_ESW_IPRES_ADDRESS(ip_priority_num);
	tmp_prio = fecp->ESW_IPRES;

	fecp->ESW_IPRES = tmp | tmp_prio;

	fecp->ESW_IPRES = MCF_ESW_IPRES_READ |
		MCF_ESW_IPRES_ADDRESS(ip_priority_num);
	tmp_prio = fecp->ESW_IPRES;

	/* enable port  IP priority lookup function*/
	fecp->ESW_PRES[port] |= MCF_ESW_PRES_IP;
	return 0;
}

static int esw_framecalssify_mac_priority_lookup(
		struct switch_enet_private *fep, int port)
{
	volatile switch_t  *fecp;

	if ((port < 0) || (port > 3)) {
		printk(KERN_ERR "%s: do not support the port %d\n",
			__func__, port);
		return -1;
	}

	fecp = fep->hwp;
	fecp->ESW_PRES[port] |= MCF_ESW_PRES_MAC;

	return 0;
}

static int esw_frame_calssify_priority_init(struct switch_enet_private *fep,
	int port, unsigned char priority_value)
{
	volatile switch_t  *fecp;

	fecp = fep->hwp;

	if ((port < 0) || (port > 3)) {
		printk(KERN_ERR "%s: do not support the port %d\n",
			__func__, port);
		return -1;
	}
	/*disable all priority lookup function*/
	fecp->ESW_PRES[port] = 0;
	fecp->ESW_PRES[port] = MCF_ESW_PRES_DFLT_PRI(priority_value & 0x7);

	return 0;
}

/*---------------------------------------------------------------------------*/
static int esw_get_statistics_status(struct switch_enet_private *fep,
	esw_statistics_status *pStatistics)
{
	volatile switch_t  *fecp;
	fecp = fep->hwp;

	pStatistics->ESW_DISCN   = fecp->ESW_DISCN;
	pStatistics->ESW_DISCB   = fecp->ESW_DISCB;
	pStatistics->ESW_NDISCN  = fecp->ESW_NDISCN;
	pStatistics->ESW_NDISCB  = fecp->ESW_NDISCB;
	return 0;
}

static int esw_get_port_statistics_status(struct switch_enet_private *fep,
	int port, esw_port_statistics_status *pPortStatistics)
{
	volatile switch_t  *fecp;

	if ((port < 0) || (port > 3)) {
		printk(KERN_ERR "%s: do not support the port %d\n",
			__func__, port);
		return -1;
	}

	fecp = fep->hwp;

	pPortStatistics->MCF_ESW_POQC   =
		fecp->port_statistics_status[port].MCF_ESW_POQC;
	pPortStatistics->MCF_ESW_PMVID  =
		fecp->port_statistics_status[port].MCF_ESW_PMVID;
	pPortStatistics->MCF_ESW_PMVTAG =
		fecp->port_statistics_status[port].MCF_ESW_PMVTAG;
	pPortStatistics->MCF_ESW_PBL    =
		fecp->port_statistics_status[port].MCF_ESW_PBL;
	return 0;
}
/*----------------------------------------------------------------------*/
static int esw_get_output_queue_status(struct switch_enet_private *fep,
	esw_output_queue_status *pOutputQueue)
{
	volatile switch_t  *fecp;

	fecp = fep->hwp;
	pOutputQueue->ESW_MMSR  = fecp->ESW_MMSR;
	pOutputQueue->ESW_LMT   = fecp->ESW_LMT;
	pOutputQueue->ESW_LFC   = fecp->ESW_LFC;
	pOutputQueue->ESW_IOSR  = fecp->ESW_IOSR;
	pOutputQueue->ESW_PCSR  = fecp->ESW_PCSR;
	pOutputQueue->ESW_QWT   = fecp->ESW_QWT;
	pOutputQueue->ESW_P0BCT = fecp->ESW_P0BCT;
	return 0;
}

/* set output queue memory status and configure*/
static int esw_set_output_queue_memory(struct switch_enet_private *fep,
	int fun_num, esw_output_queue_status *pOutputQueue)
{
	volatile switch_t  *fecp;

	fecp = fep->hwp;

	if (fun_num == 1) {
		/* memory manager status*/
		fecp->ESW_MMSR = pOutputQueue->ESW_MMSR;
	} else if (fun_num == 2) {
		/*low memory threshold*/
		fecp->ESW_LMT = pOutputQueue->ESW_LMT;
	} else if (fun_num == 3) {
		/*lowest number of free cells*/
		fecp->ESW_LFC = pOutputQueue->ESW_LFC;
	} else if (fun_num == 4) {
		/*queue weights*/
		fecp->ESW_QWT = pOutputQueue->ESW_QWT;
	} else if (fun_num == 5) {
		/*port 0 backpressure congenstion thresled*/
		fecp->ESW_P0BCT = pOutputQueue->ESW_P0BCT;
	} else {
		printk(KERN_ERR "%s: do not support the cmd %x\n",
			__func__, fun_num);
		return -1;
	}
	return 0;
}

/* ----------------------------------------------------------------------- */
static void esw_get_switch_mode(struct switch_enet_private *fep,
	unsigned long *ulModeConfig)
{
	volatile switch_t  *fecp;

	fecp = fep->hwp;
	*ulModeConfig = fecp->ESW_MODE;
}

static void esw_switch_mode_configure(struct switch_enet_private *fep,
	unsigned long configure)
{
	volatile switch_t  *fecp;

	fecp = fep->hwp;
	fecp->ESW_MODE |= configure;
}

static void esw_get_bridge_port(struct switch_enet_private *fep,
	unsigned long *ulBMPConfig)
{
	volatile switch_t  *fecp;

	fecp = fep->hwp;
	*ulBMPConfig = fecp->ESW_BMPC;
}

static void  esw_bridge_port_configure(struct switch_enet_private *fep,
	unsigned long configure)
{
	volatile switch_t  *fecp;

	fecp = fep->hwp;
	fecp->ESW_BMPC = configure;
}

static int esw_get_port_all_status(struct switch_enet_private *fep,
		unsigned char portnum, struct port_all_status *port_alstatus)
{
	volatile switch_t *fecp;
	unsigned long PortBlocking;
	unsigned long PortLearning;
	unsigned long VlanVerify;
	unsigned long DiscardUnknown;
	unsigned long MultiReso;
	unsigned long BroadReso;
	unsigned long FTransmit;
	unsigned long FReceive;

	fecp = fep->hwp;
	PortBlocking = fecp->ESW_BKLR & 0x0000000f;
	PortLearning = (fecp->ESW_BKLR & 0x000f0000) >> 16;
	VlanVerify = fecp->ESW_VLANV & 0x0000000f;
	DiscardUnknown = (fecp->ESW_VLANV & 0x000f0000) >> 16;
	MultiReso = fecp->ESW_DMCR & 0x0000000f;
	BroadReso = fecp->ESW_DBCR & 0x0000000f;
	FTransmit = fecp->ESW_PER & 0x0000000f;
	FReceive = (fecp->ESW_PER & 0x000f0000) >> 16;

	switch (portnum) {
	case 0:
		port_alstatus->link_status = 1;
		port_alstatus->block_status = PortBlocking & 1;
		port_alstatus->learn_status = PortLearning & 1;
		port_alstatus->vlan_verify = VlanVerify & 1;
		port_alstatus->discard_unknown = DiscardUnknown & 1;
		port_alstatus->multi_reso = MultiReso & 1;
		port_alstatus->broad_reso = BroadReso & 1;
		port_alstatus->ftransmit = FTransmit & 1;
		port_alstatus->freceive = FReceive & 1;
		break;
	case 1:
		port_alstatus->link_status =
			ports_link_status.port1_link_status;
		port_alstatus->block_status = (PortBlocking >> 1) & 1;
		port_alstatus->learn_status = (PortLearning >> 1) & 1;
		port_alstatus->vlan_verify = (VlanVerify >> 1) & 1;
		port_alstatus->discard_unknown = (DiscardUnknown >> 1) & 1;
		port_alstatus->multi_reso = (MultiReso >> 1) & 1;
		port_alstatus->broad_reso = (BroadReso >> 1) & 1;
		port_alstatus->ftransmit = (FTransmit >> 1) & 1;
		port_alstatus->freceive = (FReceive >> 1) & 1;
		break;
	case 2:
		port_alstatus->link_status =
			ports_link_status.port2_link_status;
		port_alstatus->block_status = (PortBlocking >> 2) & 1;
		port_alstatus->learn_status = (PortLearning >> 2) & 1;
		port_alstatus->vlan_verify = (VlanVerify >> 2) & 1;
		port_alstatus->discard_unknown = (DiscardUnknown >> 2) & 1;
		port_alstatus->multi_reso = (MultiReso >> 2) & 1;
		port_alstatus->broad_reso = (BroadReso >> 2) & 1;
		port_alstatus->ftransmit = (FTransmit >> 2) & 1;
		port_alstatus->freceive = (FReceive >> 2) & 1;
		break;
	default:
		printk(KERN_ERR "%s:do not support the port %d",
					__func__, portnum);
		break;
	}
	return 0;
}

static int esw_atable_get_entry_port_number(struct switch_enet_private *fep,
		unsigned char *mac_addr, unsigned char *port)
{
	int block_index, block_index_end, entry;
	unsigned long read_lo, read_hi;
	unsigned long mac_addr_lo, mac_addr_hi;

	mac_addr_lo = (unsigned long)((mac_addr[3]<<24) | (mac_addr[2]<<16) |
		(mac_addr[1]<<8) | mac_addr[0]);
	mac_addr_hi = (unsigned long)((mac_addr[5]<<8) | (mac_addr[4]));

	block_index = GET_BLOCK_PTR(crc8_calc(mac_addr));
	block_index_end = block_index + ATABLE_ENTRY_PER_SLOT;

	/* now search all the entries in the selected block */
	for (entry = block_index; entry < block_index_end; entry++) {
		read_atable(fep, entry, &read_lo, &read_hi);
		if ((read_lo == mac_addr_lo) &&
			((read_hi & 0x0000ffff) ==
			 (mac_addr_hi & 0x0000ffff))) {
			/* found the correct address */
			if ((read_hi & (1 << 16)) && (!(read_hi & (1 << 17))))
				*port = AT_EXTRACT_PORT(read_hi);
			break;
		} else
			*port = -1;
	}

	return 0;
}

static int esw_get_mac_address_lookup_table(struct switch_enet_private *fep,
	unsigned long *tableaddr, unsigned long *dnum, unsigned long *snum)
{
	unsigned long read_lo, read_hi;
	unsigned long entry;
	unsigned long dennum = 0;
	unsigned long sennum = 0;

	for (entry = 0; entry < ESW_ATABLE_MEM_NUM_ENTRIES; entry++) {
		read_atable(fep, entry, &read_lo, &read_hi);
		if ((read_hi & (1 << 17)) && (read_hi & (1 << 16))) {
			/* static entry */
			*(tableaddr + (2047 - sennum) * 11) = entry;
			*(tableaddr + (2047 - sennum) * 11 + 2) =
				read_lo & 0x000000ff;
			*(tableaddr + (2047 - sennum) * 11 + 3) =
				(read_lo & 0x0000ff00) >> 8;
			*(tableaddr + (2047 - sennum) * 11 + 4) =
				(read_lo & 0x00ff0000) >> 16;
			*(tableaddr + (2047 - sennum) * 11 + 5) =
				(read_lo & 0xff000000) >> 24;
			*(tableaddr + (2047 - sennum) * 11 + 6) =
				read_hi & 0x000000ff;
			*(tableaddr + (2047 - sennum) * 11 + 7) =
				(read_hi & 0x0000ff00) >> 8;
			*(tableaddr + (2047 - sennum) * 11 + 8) =
				AT_EXTRACT_PORTMASK(read_hi);
			*(tableaddr + (2047 - sennum) * 11 + 9) =
				AT_EXTRACT_PRIO(read_hi);
			sennum++;
		} else if ((read_hi & (1 << 16)) && (!(read_hi & (1 << 17)))) {
			/* dynamic entry */
			*(tableaddr + dennum * 11) = entry;
			*(tableaddr + dennum * 11 + 2) = read_lo & 0xff;
			*(tableaddr + dennum * 11 + 3) =
				(read_lo & 0x0000ff00) >> 8;
			*(tableaddr + dennum * 11 + 4) =
				(read_lo & 0x00ff0000) >> 16;
			*(tableaddr + dennum * 11 + 5) =
				(read_lo & 0xff000000) >> 24;
			*(tableaddr + dennum * 11 + 6) = read_hi & 0xff;
			*(tableaddr + dennum * 11 + 7) =
				(read_hi & 0x0000ff00) >> 8;
			*(tableaddr + dennum * 11 + 8) =
				AT_EXTRACT_PORT(read_hi);
			*(tableaddr + dennum * 11 + 9) =
				AT_EXTRACT_TIMESTAMP(read_hi);
			dennum++;
		}
	}

	*dnum = dennum;
	*snum = sennum;
	return 0;
}

/*----------------------------------------------------------------------------*/
/* The timer should create an interrupt every 4 seconds*/
static void l2switch_aging_timer(struct timer_list *t)
{
	struct switch_enet_private *fep = from_timer(fep, t, timer_aging);

	if (fep) {
		TIMEINCREMENT(fep->currTime);
		fep->timeChanged++;
	}

	mod_timer(&fep->timer_aging, jiffies + LEARNING_AGING_TIMER);
}

/*----------------------------------------------------------------*/
static int switch_enet_learning(void *arg)
{
	struct switch_enet_private *fep = arg;
	volatile switch_t  *fecp;

	fecp = fep->hwp;
	while (!kthread_should_stop()) {
		set_current_state(TASK_INTERRUPTIBLE);

		/* check learning record valid */
		if (fecp->ESW_LSR)
			esw_atable_dynamicms_learn_migration(fep,
					fep->currTime);
		else
			schedule_timeout(HZ/100);
	}

	return 0;
}

static int switch_enet_ioctl(struct net_device *dev,
		struct ifreq *ifr, int cmd)
{
	struct switch_enet_private *fep = netdev_priv(dev);
	volatile switch_t *fecp;
	int ret = 0;

	fecp = (volatile switch_t *)dev->base_addr;

	switch (cmd) {
	/*------------------------------------------------------------*/
	case ESW_SET_PORTENABLE_CONF:
	{
		eswIoctlPortEnableConfig configData;
		ret = copy_from_user(&configData,
			ifr->ifr_data,
			sizeof(eswIoctlPortEnableConfig));
		if (ret)
			return -EFAULT;

		ret = esw_port_enable_config(fep,
			configData.port,
			configData.tx_enable,
			configData.rx_enable);
	}
		break;
	case ESW_SET_BROADCAST_CONF:
	{
		eswIoctlPortConfig configData;
		ret = copy_from_user(&configData,
			ifr->ifr_data, sizeof(eswIoctlPortConfig));
		if (ret)
			return -EFAULT;

		ret = esw_port_broadcast_config(fep,
			configData.port, configData.enable);
	}
		break;

	case ESW_SET_MULTICAST_CONF:
	{
		eswIoctlPortConfig configData;
		ret = copy_from_user(&configData,
			ifr->ifr_data, sizeof(eswIoctlPortConfig));
		if (ret)
			return -EFAULT;

		ret = esw_port_multicast_config(fep,
			configData.port, configData.enable);
	}
		break;

	case ESW_SET_BLOCKING_CONF:
	{
		eswIoctlPortConfig configData;
		ret = copy_from_user(&configData,
			ifr->ifr_data, sizeof(eswIoctlPortConfig));

		if (ret)
			return -EFAULT;

		ret = esw_port_blocking_config(fep,
			configData.port, configData.enable);
	}
		break;

	case ESW_SET_LEARNING_CONF:
	{
		eswIoctlPortConfig configData;

		ret = copy_from_user(&configData,
			ifr->ifr_data, sizeof(eswIoctlPortConfig));
		if (ret)
			return -EFAULT;

		ret = esw_port_learning_config(fep,
			configData.port, configData.enable);
	}
		break;

	case ESW_SET_PORT_ENTRY_EMPTY:
	{
		unsigned long portnum;

		ret = copy_from_user(&portnum,
			ifr->ifr_data, sizeof(portnum));
		if (ret)
			return -EFAULT;
		esw_atable_dynamicms_del_entries_for_port(fep, portnum);
	}
		break;

	case ESW_SET_OTHER_PORT_ENTRY_EMPTY:
	{
		unsigned long portnum;

		ret = copy_from_user(&portnum,
			ifr->ifr_data, sizeof(portnum));
		if (ret)
			return -EFAULT;

		esw_atable_dynamicms_del_entries_for_other_port(fep, portnum);
	}
		break;

	case ESW_SET_IP_SNOOP_CONF:
	{
		eswIoctlIpsnoopConfig configData;

		ret = copy_from_user(&configData,
			ifr->ifr_data, sizeof(eswIoctlIpsnoopConfig));
		if (ret)
			return -EFAULT;

		ret = esw_ip_snoop_config(fep, configData.mode,
				configData.ip_header_protocol);
	}
		break;

	case ESW_SET_PORT_SNOOP_CONF:
	{
		eswIoctlPortsnoopConfig configData;

		ret = copy_from_user(&configData,
			ifr->ifr_data, sizeof(eswIoctlPortsnoopConfig));
		if (ret)
			return -EFAULT;

		ret = esw_tcpudp_port_snoop_config(fep, configData.mode,
				configData.compare_port,
				configData.compare_num);
	}
		break;

	case ESW_SET_PORT_MIRROR_CONF_PORT_MATCH:
	{
		struct eswIoctlMirrorCfgPortMatch configData;

		ret = copy_from_user(&configData,
			ifr->ifr_data, sizeof(configData));
		if (ret)
			return -EFAULT;
		ret = esw_port_mirroring_config_port_match(fep,
			configData.mirror_port, configData.port_match_en,
			configData.port);
	}
		break;

	case ESW_SET_PORT_MIRROR_CONF:
	{
		eswIoctlPortMirrorConfig configData;

		ret = copy_from_user(&configData,
			ifr->ifr_data, sizeof(eswIoctlPortMirrorConfig));
		if (ret)
			return -EFAULT;

		ret = esw_port_mirroring_config(fep,
			configData.mirror_port, configData.port,
			configData.mirror_enable,
			configData.src_mac, configData.des_mac,
			configData.egress_en, configData.ingress_en,
			configData.egress_mac_src_en,
			configData.egress_mac_des_en,
			configData.ingress_mac_src_en,
			configData.ingress_mac_des_en);
	}
		break;

	case ESW_SET_PORT_MIRROR_CONF_ADDR_MATCH:
	{
		struct eswIoctlMirrorCfgAddrMatch configData;

		ret = copy_from_user(&configData,
			ifr->ifr_data, sizeof(configData));
		if (ret)
			return -EFAULT;

		ret = esw_port_mirroring_config_addr_match(fep,
			configData.mirror_port, configData.addr_match_en,
			configData.mac_addr);
	}
		break;

	case ESW_SET_PIRORITY_VLAN:
	{
		eswIoctlPriorityVlanConfig configData;

		ret = copy_from_user(&configData,
			ifr->ifr_data, sizeof(eswIoctlPriorityVlanConfig));
		if (ret)
			return -EFAULT;

		ret = esw_framecalssify_vlan_priority_lookup(fep,
			configData.port, configData.func_enable,
			configData.vlan_pri_table_num,
			configData.vlan_pri_table_value);
	}
		break;

	case ESW_SET_PIRORITY_IP:
	{
		eswIoctlPriorityIPConfig configData;

		ret = copy_from_user(&configData,
			ifr->ifr_data, sizeof(eswIoctlPriorityIPConfig));
		if (ret)
			return -EFAULT;

		ret = esw_framecalssify_ip_priority_lookup(fep,
			configData.port, configData.func_enable,
			configData.ipv4_en, configData.ip_priority_num,
			configData.ip_priority_value);
	}
		break;

	case ESW_SET_PIRORITY_MAC:
	{
		eswIoctlPriorityMacConfig configData;

		ret = copy_from_user(&configData,
			ifr->ifr_data, sizeof(eswIoctlPriorityMacConfig));
		if (ret)
			return -EFAULT;

		ret = esw_framecalssify_mac_priority_lookup(fep,
			configData.port);
	}
		break;

	case ESW_SET_PIRORITY_DEFAULT:
	{
		eswIoctlPriorityDefaultConfig configData;

		ret = copy_from_user(&configData,
			ifr->ifr_data, sizeof(eswIoctlPriorityDefaultConfig));
		if (ret)
			return -EFAULT;

		ret = esw_frame_calssify_priority_init(fep,
			configData.port, configData.priority_value);
	}
		break;

	case ESW_SET_P0_FORCED_FORWARD:
	{
		eswIoctlP0ForcedForwardConfig configData;

		ret = copy_from_user(&configData,
			ifr->ifr_data, sizeof(eswIoctlP0ForcedForwardConfig));
		if (ret)
			return -EFAULT;

		ret = esw_forced_forward(fep, configData.port1,
			configData.port2, configData.enable);
	}
		break;

	case ESW_SET_BRIDGE_CONFIG:
	{
		unsigned long configData;

		ret = copy_from_user(&configData,
			ifr->ifr_data, sizeof(unsigned long));
		if (ret)
			return -EFAULT;

		esw_bridge_port_configure(fep, configData);
	}
		break;

	case ESW_SET_SWITCH_MODE:
	{
		unsigned long configData;

		ret = copy_from_user(&configData,
			ifr->ifr_data, sizeof(unsigned long));
		if (ret)
			return -EFAULT;

		esw_switch_mode_configure(fep, configData);
	}
		break;

	case ESW_SET_OUTPUT_QUEUE_MEMORY:
	{
		eswIoctlOutputQueue configData;

		ret = copy_from_user(&configData,
			ifr->ifr_data, sizeof(eswIoctlOutputQueue));
		if (ret)
			return -EFAULT;

		ret = esw_set_output_queue_memory(fep,
			configData.fun_num, &configData.sOutputQueue);
	}
		break;

	case ESW_SET_VLAN_OUTPUT_PROCESS:
	{
		eswIoctlVlanOutputConfig configData;

		ret = copy_from_user(&configData,
			ifr->ifr_data, sizeof(eswIoctlVlanOutputConfig));
		if (ret)
			return -EFAULT;

		ret = esw_vlan_output_process(fep,
			configData.port, configData.mode);
	}
		break;

	case ESW_SET_VLAN_INPUT_PROCESS:
	{
		eswIoctlVlanInputConfig configData;

		ret = copy_from_user(&configData,
			ifr->ifr_data,
			sizeof(eswIoctlVlanInputConfig));
		if (ret)
			return -EFAULT;

		ret = esw_vlan_input_process(fep, configData.port,
				configData.mode, configData.port_vlanid);
	}
		break;

	case ESW_SET_VLAN_DOMAIN_VERIFICATION:
	{
		eswIoctlVlanVerificationConfig configData;

		ret = copy_from_user(&configData,
			ifr->ifr_data,
			sizeof(eswIoctlVlanVerificationConfig));
		if (ret)
			return -EFAULT;

		ret = esw_set_vlan_verification(
			fep, configData.port,
			configData.vlan_domain_verify_en,
			configData.vlan_discard_unknown_en);
	}
		break;

	case ESW_SET_VLAN_RESOLUTION_TABLE:
	{
		eswIoctlVlanResoultionTable configData;

		ret = copy_from_user(&configData,
			ifr->ifr_data,
			sizeof(eswIoctlVlanResoultionTable));
		if (ret)
			return -EFAULT;

		ret = esw_set_vlan_resolution_table(
			fep, configData.port_vlanid,
			configData.vlan_domain_num,
			configData.vlan_domain_port);

	}
		break;

	case ESW_SET_VLAN_ID:
	{
		unsigned long configData;
		ret = copy_from_user(&configData, ifr->ifr_data,
				sizeof(configData));
		if (ret)
			return -EFAULT;

		ret = esw_set_vlan_id(fep, configData);
	}
		break;

	case ESW_SET_VLAN_ID_CLEARED:
	{
		unsigned long configData;
		ret = copy_from_user(&configData, ifr->ifr_data,
				sizeof(configData));
		if (ret)
			return -EFAULT;

		ret = esw_set_vlan_id_cleared(fep, configData);
	}
		break;

	case ESW_SET_PORT_IN_VLAN_ID:
	{
		eswIoctlVlanResoultionTable configData;

		ret = copy_from_user(&configData, ifr->ifr_data,
				sizeof(configData));
		if (ret)
			return -EFAULT;

		ret = esw_set_port_in_vlan_id(fep, configData);
	}
		break;

	/*--------------------------------------------------------------------*/
	case ESW_UPDATE_STATIC_MACTABLE:
	{
		eswIoctlUpdateStaticMACtable configData;

		ret = copy_from_user(&configData,
			ifr->ifr_data, sizeof(eswIoctlUpdateStaticMACtable));
		if (ret)
			return -EFAULT;

		ret = esw_update_atable_static(configData.mac_addr,
				configData.port, configData.priority, fep);
	}
		break;

	case ESW_CLEAR_ALL_MACTABLE:
	{
		esw_clear_atable(fep);
	}
		break;

	/*-------------------get----------------------------------------------*/
	case ESW_GET_STATISTICS_STATUS:
	{
		esw_statistics_status Statistics;
		esw_port_statistics_status PortSta;
		int i;

		ret = esw_get_statistics_status(fep, &Statistics);
		if (ret != 0) {
			printk(KERN_ERR "%s: cmd %x fail\n", __func__, cmd);
			return -1;
		}
		printk(KERN_INFO "DISCN : %10ld      DISCB : %10ld\n",
				Statistics.ESW_DISCN, Statistics.ESW_DISCB);
		printk(KERN_INFO "NDISCN: %10ld      NDISCB: %10ld\n",
				Statistics.ESW_NDISCN, Statistics.ESW_NDISCB);

		for (i = 0; i < 3; i++) {
			ret = esw_get_port_statistics_status(fep, i,
					&PortSta);
			if (ret != 0) {
				printk(KERN_ERR "%s: cmd %x fail\n",
					__func__, cmd);
				return -1;
			}
			printk(KERN_INFO "port %d:  POQC  : %ld\n",
					i, PortSta.MCF_ESW_POQC);
			printk(KERN_INFO "         PMVID : %ld\n",
					PortSta.MCF_ESW_PMVID);
			printk(KERN_INFO "	 PMVTAG: %ld\n",
					PortSta.MCF_ESW_PMVTAG);
			printk(KERN_INFO "	 PBL   : %ld\n",
					PortSta.MCF_ESW_PBL);
		}
	}
		break;

	case ESW_GET_LEARNING_CONF:
	{
		unsigned long PortLearning;

		esw_get_port_learning(fep, &PortLearning);
		ret = copy_to_user(ifr->ifr_data, &PortLearning,
			sizeof(unsigned long));
		if (ret)
			return -EFAULT;
	}
		break;

	case ESW_GET_BLOCKING_CONF:
	{
		unsigned long PortBlocking;

		esw_get_port_blocking(fep, &PortBlocking);
		ret = copy_to_user(ifr->ifr_data, &PortBlocking,
			sizeof(unsigned long));
		if (ret)
			return -EFAULT;
	}
		break;

	case ESW_GET_MULTICAST_CONF:
	{
		unsigned long PortMulticast;

		esw_get_port_multicast(fep, &PortMulticast);
		ret = copy_to_user(ifr->ifr_data, &PortMulticast,
			sizeof(unsigned long));
		if (ret)
			return -EFAULT;
	}
		break;

	case ESW_GET_BROADCAST_CONF:
	{
		unsigned long PortBroadcast;

		esw_get_port_broadcast(fep, &PortBroadcast);
		ret = copy_to_user(ifr->ifr_data, &PortBroadcast,
		sizeof(unsigned long));
		if (ret)
			return -EFAULT;
	}
		break;

	case ESW_GET_PORTENABLE_CONF:
	{
		unsigned long PortEnable;

		esw_get_port_enable(fep, &PortEnable);
		ret = copy_to_user(ifr->ifr_data, &PortEnable,
			sizeof(unsigned long));
		if (ret)
			return -EFAULT;
	}
		break;

	case ESW_GET_IP_SNOOP_CONF:
	{
		unsigned long ESW_IPSNP[8];
		int i;

		esw_get_ip_snoop_config(fep, (unsigned long *)ESW_IPSNP);
		printk(KERN_INFO "IP Protocol     Mode     Type\n");
		for (i = 0; i < 8; i++) {
			if (ESW_IPSNP[i] != 0)
				printk(KERN_INFO "%3ld             "
					"%1ld        %s\n",
					(ESW_IPSNP[i] >> 8) & 0xff,
					(ESW_IPSNP[i] >> 1) & 3,
					ESW_IPSNP[i] & 1 ? "Active" :
					"Inactive");
		}
	}
		break;

	case ESW_GET_PORT_SNOOP_CONF:
	{
		unsigned long ESW_PSNP[8];
		int i;

		esw_get_tcpudp_port_snoop_config(fep,
				(unsigned long *)ESW_PSNP);
		printk(KERN_INFO "TCP/UDP Port  SrcCompare  DesCompare  "
				"Mode  Type\n");
		for (i = 0; i < 8; i++) {
			if (ESW_PSNP[i] != 0)
				printk(KERN_INFO "%5ld         %s           "
					"%s           %1ld     %s\n",
					(ESW_PSNP[i] >> 16) & 0xffff,
					(ESW_PSNP[i] >> 4) & 1 ? "Y" : "N",
					(ESW_PSNP[i] >> 3) & 1 ? "Y" : "N",
					(ESW_PSNP[i] >> 1) & 3,
					ESW_PSNP[i] & 1 ? "Active" :
					"Inactive");
		}
	}
		break;

	case ESW_GET_PORT_MIRROR_CONF:
		esw_get_port_mirroring(fep);
		break;

	case ESW_GET_P0_FORCED_FORWARD:
	{
		unsigned long ForceForward;

		esw_get_forced_forward(fep, &ForceForward);
		ret = copy_to_user(ifr->ifr_data, &ForceForward,
			sizeof(unsigned long));
		if (ret)
			return -EFAULT;
	}
		break;

	case ESW_GET_SWITCH_MODE:
	{
		unsigned long Config;

		esw_get_switch_mode(fep, &Config);
		ret = copy_to_user(ifr->ifr_data, &Config,
			sizeof(unsigned long));
		if (ret)
			return -EFAULT;
	}
		break;

	case ESW_GET_BRIDGE_CONFIG:
	{
		unsigned long Config;

		esw_get_bridge_port(fep, &Config);
		ret = copy_to_user(ifr->ifr_data, &Config,
			sizeof(unsigned long));
		if (ret)
			return -EFAULT;
	}
		break;
	case ESW_GET_OUTPUT_QUEUE_STATUS:
	{
		esw_output_queue_status Config;
		esw_get_output_queue_status(fep,
			&Config);
		ret = copy_to_user(ifr->ifr_data, &Config,
			sizeof(esw_output_queue_status));
		if (ret)
			return -EFAULT;
	}
		break;

	case ESW_GET_VLAN_OUTPUT_PROCESS:
	{
		unsigned long Config;
		int tmp;
		int i;

		esw_get_vlan_output_config(fep, &Config);

		for (i = 0; i < 3; i++) {
			tmp = (Config >> (i << 1)) & 3;

			if (tmp != 0)
				printk(KERN_INFO "port %d: vlan output "
					"manipulation enable (mode %d)\n",
					i, tmp);
			else
				printk(KERN_INFO "port %d: vlan output "
					"manipulation disable\n", i);
		}
	}
		break;

	case ESW_GET_VLAN_INPUT_PROCESS:
	{
		eswIoctlVlanInputStatus Config;
		int i;

		esw_get_vlan_input_config(fep, &Config);

		for (i = 0; i < 3; i++) {
			if (((Config.ESW_VIMEN >> i) & 1) == 0)
				printk(KERN_INFO "port %d: vlan input "
						"manipulation disable\n", i);
			else
				printk("port %d: vlan input manipulation enable"
					" (mode %ld, vlan id %ld)\n", i,
					(((Config.ESW_VIMSEL >> (i << 1)) & 3)
					 + 1), Config.ESW_PID[i]);
		}
	}
		break;

	case ESW_GET_VLAN_RESOLUTION_TABLE:
	{
		struct eswVlanTableItem vtableitem;
		unsigned char tmp0, tmp1, tmp2;
		int i;

		esw_get_vlan_resolution_table(fep, &vtableitem);

		printk(KERN_INFO "VLAN Name      VLAN Id      Ports\n");
		for (i = 0; i < vtableitem.valid_num; i++) {
			tmp0 = vtableitem.table[i].vlan_domain_port & 1;
			tmp1 = (vtableitem.table[i].vlan_domain_port >> 1) & 1;
			tmp2 = (vtableitem.table[i].vlan_domain_port >> 2) & 1;
			printk(KERN_INFO "%2d             %4d         %s%s%s\n",
				i, vtableitem.table[i].port_vlanid,
				tmp0 ? "0 " : "", tmp1 ? "1 " : "",
				tmp2 ? "2" : "");
		}
	}
		break;

	case ESW_GET_VLAN_DOMAIN_VERIFICATION:
	{
		unsigned long Config;

		esw_get_vlan_verification(fep, &Config);
		ret = copy_to_user(ifr->ifr_data, &Config,
			sizeof(unsigned long));
		if (ret)
			return -EFAULT;
	}
		break;

	case ESW_GET_ENTRY_PORT_NUMBER:
	{
		unsigned char mac_addr[6];
		unsigned char portnum;

		ret = copy_from_user(mac_addr,
			ifr->ifr_data, sizeof(mac_addr));
		if (ret)
			return -EFAULT;

		ret = esw_atable_get_entry_port_number(fep, mac_addr,
				&portnum);

		ret = copy_to_user(ifr->ifr_data, &portnum,
				sizeof(unsigned char));
		if (ret)
			return -EFAULT;
	}
		break;

	case ESW_GET_LOOKUP_TABLE:
	{
		unsigned long *ConfigData;
		unsigned long dennum, sennum;
		int i;
		int tmp;

		ConfigData = kmalloc(sizeof(struct eswAddrTableEntryExample) *
				ESW_ATABLE_MEM_NUM_ENTRIES, GFP_KERNEL);
		ret = esw_get_mac_address_lookup_table(fep, ConfigData,
				&dennum, &sennum);
		printk(KERN_INFO "Dynamic entries number: %ld\n", dennum);
		printk(KERN_INFO "Static entries number: %ld\n", sennum);
		printk(KERN_INFO "Type      MAC address         Port   Timestamp\n");
		for (i = 0; i < dennum; i++) {
			printk(KERN_INFO "dynamic   "
				"%02lx-%02lx-%02lx-%02lx-%02lx-%02lx   "
				"%01lx      %4ld\n", *(ConfigData + i * 11 + 2),
				*(ConfigData + i * 11 + 3),
				*(ConfigData + i * 11 + 4),
				*(ConfigData + i * 11 + 5),
				*(ConfigData + i * 11 + 6),
				*(ConfigData + i * 11 + 7),
				*(ConfigData + i * 11 + 8),
				*(ConfigData + i * 11 + 9));
		}

		if (sennum != 0)
			printk(KERN_INFO "Type      MAC address"
					"         Port   Priority\n");

		for (i = 0; i < sennum; i++) {
			printk(KERN_INFO "static    %02lx-%02lx-%02lx-%02lx"
					"-%02lx-%02lx   ",
					*(ConfigData + (2047 - i) * 11 + 2),
					*(ConfigData + (2047 - i) * 11 + 3),
					*(ConfigData + (2047 - i) * 11 + 4),
					*(ConfigData + (2047 - i) * 11 + 5),
					*(ConfigData + (2047 - i) * 11 + 6),
					*(ConfigData + (2047 - i) * 11 + 7));

			tmp = *(ConfigData + (2047 - i) * 11 + 8);
			if ((tmp == 0) || (tmp == 2) || (tmp == 4))
				printk("%01x      ", tmp >> 1);
			else if (tmp == 3)
				printk("0,1    ");
			else if (tmp == 5)
				printk("0,2    ");
			else if (tmp == 6)
				printk("1,2    ");

			printk("%4ld\n", *(ConfigData + (2047 - i) * 11 + 9));
		}
		kfree(ConfigData);
	}
		break;

	case ESW_GET_PORT_STATUS:
	{
		unsigned long PortBlocking;

		esw_get_port_blocking(fep, &PortBlocking);

		ports_link_status.port0_block_status = PortBlocking & 1;
		ports_link_status.port1_block_status = (PortBlocking >> 1) & 1;
		ports_link_status.port2_block_status = PortBlocking >> 2;

		ret = copy_to_user(ifr->ifr_data, &ports_link_status,
				sizeof(ports_link_status));
		if (ret)
			return -EFAULT;
	}
		break;

	case ESW_GET_PORT_ALL_STATUS:
	{
		unsigned char portnum;
		struct port_all_status port_astatus;

		ret = copy_from_user(&portnum,
			ifr->ifr_data, sizeof(portnum));
		if (ret)
			return -EFAULT;

		esw_get_port_all_status(fep, portnum, &port_astatus);
		printk(KERN_INFO "Port %d status:\n", portnum);
		printk(KERN_INFO "Link:%-4s          Blocking:%1s          "
			"Learning:%1s\n",
			port_astatus.link_status ? "Up" : "Down",
			port_astatus.block_status ? "Y" : "N",
			port_astatus.learn_status ? "N" : "Y");
		printk(KERN_INFO "VLAN Verify:%1s      Discard Unknown:%1s   "
			"Multicast Res:%1s\n",
			port_astatus.vlan_verify ? "Y" : "N",
			port_astatus.discard_unknown ? "Y" : "N",
			port_astatus.multi_reso ? "Y" : "N");
		printk(KERN_INFO "Broadcast Res:%1s    Transmit:%-7s    "
			"Receive:%7s\n",
			port_astatus.broad_reso ? "Y" : "N",
			port_astatus.ftransmit ? "Enable" : "Disable",
			port_astatus.freceive ? "Enable" : "Disable");

	}
		break;

	case ESW_GET_USER_PID:
	{
		long get_pid = 0;
		ret = copy_from_user(&get_pid,
			ifr->ifr_data, sizeof(get_pid));

		if (ret)
			return -EFAULT;
		user_pid = get_pid;
	}
		break;
	/*------------------------------------------------------------------*/
	default:
		return -EOPNOTSUPP;
	}

	return ret;
}

static netdev_tx_t switch_enet_start_xmit(struct sk_buff *skb,
				struct net_device *dev)
{
	struct switch_enet_private *fep;
	volatile switch_t	*fecp;
	struct netdev_queue *txq = netdev_get_tx_queue(dev, 0);
	cbd_t	*bdp;
	unsigned short	status;
	unsigned long flags;

	fep = netdev_priv(dev);
	fecp = (switch_t *)fep->hwp;

	spin_lock_irqsave(&fep->hw_lock, flags);
	/* Fill in a Tx ring entry */
	bdp = fep->cur_tx;

	status = bdp->cbd_sc;

	/* Clear all of the status flags.
	 */
	status &= ~BD_ENET_TX_STATS;

	/* Set buffer length and buffer pointer.
	*/
	bdp->cbd_bufaddr = __pa(skb->data);
	bdp->cbd_datlen = skb->len;

	/*
	 *	On some FEC implementations data must be aligned on
	 *	4-byte boundaries. Use bounce buffers to copy data
	 *	and get it aligned. Ugh.
	 */
	if (bdp->cbd_bufaddr & 0x3) {
		unsigned int index1;
		index1 = bdp - fep->tx_bd_base;

		memcpy(fep->tx_bounce[index1],
		       (void *)skb->data, bdp->cbd_datlen);
		bdp->cbd_bufaddr = __pa(fep->tx_bounce[index1]);
	}

	/* Save skb pointer. */
	fep->tx_skbuff[fep->skb_cur] = skb;

	dev->stats.tx_bytes += skb->len;
	fep->skb_cur = (fep->skb_cur+1) & TX_RING_MOD_MASK;

	/* Push the data cache so the CPM does not get stale memory
	 * data.
	 */
	flush_cf_dcache((unsigned long)skb->data,
			   (unsigned long)skb->data + skb->len);

	/* Send it on its way.  Tell FEC it's ready, interrupt when done,
	 * it's the last BD of the frame, and to put the CRC on the end.
	 */

	status |= (BD_ENET_TX_READY | BD_ENET_TX_INTR
			| BD_ENET_TX_LAST | BD_ENET_TX_TC);
	bdp->cbd_sc = status;

	if (txq->trans_start != jiffies)
		txq->trans_start = jiffies;

	/* Trigger transmission start */
	fecp->fec_x_des_active = MCF_ESW_TDAR_X_DES_ACTIVE;

	/* If this was the last BD in the ring,
	 * start at the beginning again.*/
	if (status & BD_ENET_TX_WRAP)
		bdp = fep->tx_bd_base;
	else
		bdp++;

	if (bdp == fep->dirty_tx) {
		fep->tx_full = 1;
		netif_stop_queue(dev);
		printk(KERN_ERR "%s:  net stop\n", __func__);
	}

	fep->cur_tx = (cbd_t *)bdp;

	spin_unlock_irqrestore(&fep->hw_lock, flags);

	return NETDEV_TX_OK;
}

static void switch_timeout(struct net_device *dev, unsigned int txqueue)
{
	struct switch_enet_private *fep = netdev_priv(dev);

	printk(KERN_ERR "%s: transmit timed out.\n", dev->name);
	dev->stats.tx_errors++;
	switch_restart(dev, fep->full_duplex);
	netif_wake_queue(dev);
}

/* The interrupt handler.
 * This is called from the MPC core interrupt.
 */
static irqreturn_t switch_enet_interrupt(int irq, void *dev_id)
{
	struct	net_device *dev = dev_id;
	volatile switch_t *fecp;
	uint	int_events;
	irqreturn_t ret = IRQ_NONE;

	fecp = (switch_t *)dev->base_addr;

	/* Get the interrupt events that caused us to be here.
	*/
	do {
		int_events = fecp->switch_ievent;
		fecp->switch_ievent = int_events;
		/* Handle receive event in its own function. */

		/* Transmit OK, or non-fatal error. Update the buffer
		   descriptors. Switch handles all errors, we just discover
		   them as part of the transmit process.
		*/
		if (int_events & MCF_ESW_ISR_OD0)
			ret = IRQ_HANDLED;

		if (int_events & MCF_ESW_ISR_OD1)
			ret = IRQ_HANDLED;

		if (int_events & MCF_ESW_ISR_OD2)
			ret = IRQ_HANDLED;

		if (int_events & MCF_ESW_ISR_RXB)
			ret = IRQ_HANDLED;

		if (int_events & MCF_ESW_ISR_RXF) {
			ret = IRQ_HANDLED;
			switch_enet_rx(dev);
		}

		if (int_events & MCF_ESW_ISR_TXB)
			ret = IRQ_HANDLED;

		if (int_events & MCF_ESW_ISR_TXF) {
			ret = IRQ_HANDLED;
			switch_enet_tx(dev);
		}

	} while (int_events);

	return ret;
}

static void switch_enet_tx(struct net_device *dev)
{
	struct	switch_enet_private *fep;
	cbd_t	*bdp;
	unsigned short status;
	struct	sk_buff	*skb;

	fep = netdev_priv(dev);
	spin_lock_irq(&fep->hw_lock);
	bdp = fep->dirty_tx;

	while (((status = bdp->cbd_sc) & BD_ENET_TX_READY) == 0) {
		if (bdp == fep->cur_tx && fep->tx_full == 0)
			break;

		skb = fep->tx_skbuff[fep->skb_dirty];
		/* Check for errors. */
		if (status & (BD_ENET_TX_HB | BD_ENET_TX_LC |
				   BD_ENET_TX_RL | BD_ENET_TX_UN |
				   BD_ENET_TX_CSL)) {
			dev->stats.tx_errors++;
			if (status & BD_ENET_TX_HB)  /* No heartbeat */
				dev->stats.tx_heartbeat_errors++;
			if (status & BD_ENET_TX_LC)  /* Late collision */
				dev->stats.tx_window_errors++;
			if (status & BD_ENET_TX_RL)  /* Retrans limit */
				dev->stats.tx_aborted_errors++;
			if (status & BD_ENET_TX_UN)  /* Underrun */
				dev->stats.tx_fifo_errors++;
			if (status & BD_ENET_TX_CSL) /* Carrier lost */
				dev->stats.tx_carrier_errors++;
		} else {
			dev->stats.tx_packets++;
		}

		/* Deferred means some collisions occurred during transmit,
		 * but we eventually sent the packet OK.
		 */
		if (status & BD_ENET_TX_DEF)
			dev->stats.collisions++;

		/* Free the sk buffer associated with this last transmit.
		 */
		dev_kfree_skb_any(skb);
		fep->tx_skbuff[fep->skb_dirty] = NULL;
		fep->skb_dirty = (fep->skb_dirty + 1) & TX_RING_MOD_MASK;

		/* Update pointer to next buffer descriptor to be transmitted.
		 */
		if (status & BD_ENET_TX_WRAP)
			bdp = fep->tx_bd_base;
		else
			bdp++;

		/* Since we have freed up a buffer, the ring is no longer
		 * full.
		 */
		if (fep->tx_full) {
			fep->tx_full = 0;
			printk(KERN_ERR "%s: tx full is zero\n", __func__);
			if (netif_queue_stopped(dev))
				netif_wake_queue(dev);
		}
	}
	fep->dirty_tx = (cbd_t *)bdp;
	spin_unlock_irq(&fep->hw_lock);
}


/* During a receive, the cur_rx points to the current incoming buffer.
 * When we update through the ring, if the next incoming buffer has
 * not been given to the system, we just set the empty indicator,
 * effectively tossing the packet.
 */
static void switch_enet_rx(struct net_device *dev)
{
	struct	switch_enet_private *fep;
	volatile switch_t *fecp;
	cbd_t *bdp;
	unsigned short status;
	struct	sk_buff	*skb;
	ushort	pkt_len;
	__u8 *data;

	fep = netdev_priv(dev);
	/*fecp = (volatile switch_t *)dev->base_addr;*/
	fecp = (volatile switch_t *)fep->hwp;

	spin_lock_irq(&fep->hw_lock);
	/* First, grab all of the stats for the incoming packet.
	 * These get messed up if we get called due to a busy condition.
	 */
	bdp = fep->cur_rx;

	while (!((status = bdp->cbd_sc) & BD_ENET_RX_EMPTY)) {

		/* Since we have allocated space to hold a complete frame,
		 * the last indicator should be set.
		 * */
		if ((status & BD_ENET_RX_LAST) == 0)
			printk(KERN_ERR "SWITCH ENET: rcv is not +last\n");

		if (!fep->opened)
			goto rx_processing_done;

		/* Check for errors. */
		if (status & (BD_ENET_RX_LG | BD_ENET_RX_SH | BD_ENET_RX_NO |
			   BD_ENET_RX_CR | BD_ENET_RX_OV)) {
			dev->stats.rx_errors++;
			if (status & (BD_ENET_RX_LG | BD_ENET_RX_SH)) {
				/* Frame too long or too short. */
				dev->stats.rx_length_errors++;
			}
			if (status & BD_ENET_RX_NO)	/* Frame alignment */
				dev->stats.rx_frame_errors++;
			if (status & BD_ENET_RX_CR)	/* CRC Error */
				dev->stats.rx_crc_errors++;
			if (status & BD_ENET_RX_OV)	/* FIFO overrun */
				dev->stats.rx_fifo_errors++;
		}
		/* Report late collisions as a frame error.
		 * On this error, the BD is closed, but we don't know what we
		 * have in the buffer.  So, just drop this frame on the floor.
		 * */
		if (status & BD_ENET_RX_CL) {
			dev->stats.rx_errors++;
			dev->stats.rx_frame_errors++;
			goto rx_processing_done;
		}
		/* Process the incoming frame */
		dev->stats.rx_packets++;
		pkt_len = bdp->cbd_datlen;
		dev->stats.rx_bytes += pkt_len;
		data = (__u8 *)__va(bdp->cbd_bufaddr);

		/* This does 16 byte alignment, exactly what we need.
		 * The packet length includes FCS, but we don't want to
		 * include that when passing upstream as it messes up
		 * bridging applications.
		 * */
		skb = dev_alloc_skb(pkt_len);

		if (skb == NULL)
			dev->stats.rx_dropped++;
		else {
			skb_put(skb, pkt_len);	/* Make room */
			skb_copy_to_linear_data(skb, data, pkt_len);
			skb->protocol = eth_type_trans(skb, dev);
			netif_rx(skb);
		}
rx_processing_done:

		/* Clear the status flags for this buffer */
		status &= ~BD_ENET_RX_STATS;

		/* Mark the buffer empty */
		status |= BD_ENET_RX_EMPTY;
		bdp->cbd_sc = status;

		/* Update BD pointer to next entry */
		if (status & BD_ENET_RX_WRAP)
			bdp = fep->rx_bd_base;
		else
			bdp++;

		/* Doing this here will keep the FEC running while we process
		 * incoming frames.  On a heavily loaded network, we should be
		 * able to keep up at the expense of system resources.
		 * */
		fecp->fec_r_des_active = MCF_ESW_RDAR_R_DES_ACTIVE;
	}
	fep->cur_rx = (cbd_t *)bdp;

	spin_unlock_irq(&fep->hw_lock);
}

static int fec_mdio_transfer(struct mii_bus *bus, int phy_id,
	int reg, int regval)
{
	struct net_device *dev = bus->priv;
	unsigned long   flags;
	struct switch_enet_private *fep;
	int tries = 100;
	int retval = 0;

	fep = netdev_priv(dev);
	spin_lock_irqsave(&fep->mii_lock, flags);

	regval |= phy_id << 23;

    if(bus->id[0] == '0')
    {
        MCF_FEC_MMFR0 = regval;
        
        
        /* wait for it to finish, this takes about 23 us on lite5200b */
        while (!(MCF_FEC_EIR0 & FEC_ENET_MII) && --tries)
            udelay(50);

        if (!tries) {
            printk(KERN_ERR "%s timeout\n", __func__);
            spin_unlock_irqrestore(&fep->mii_lock, flags);
            return -ETIMEDOUT;
        }

        MCF_FEC_EIR0 = FEC_ENET_MII;
        retval = MCF_FEC_MMFR0;
    }
    else
    {
        MCF_FEC_MMFR1 = regval;
                
        /* wait for it to finish, this takes about 23 us on lite5200b */
        while (!(MCF_FEC_EIR1 & FEC_ENET_MII) && --tries)
            udelay(50);

        if (!tries) {
            printk(KERN_ERR "%s timeout\n", __func__);
            spin_unlock_irqrestore(&fep->mii_lock, flags);
            return -ETIMEDOUT;
        }

        MCF_FEC_EIR1 = FEC_ENET_MII;
        retval = MCF_FEC_MMFR1;
        
    }
	spin_unlock_irqrestore(&fep->mii_lock, flags);

	return retval;
}


static int coldfire_fec_mdio_read(struct mii_bus *bus,
	int phy_id, int reg)
{
	int ret;
	ret = fec_mdio_transfer(bus, phy_id, reg,
		mk_mii_read(reg));
	return ret;
}

static int coldfire_fec_mdio_write(struct mii_bus *bus,
	int phy_id, int reg, u16 data)
{
	return fec_mdio_transfer(bus, phy_id, reg,
			mk_mii_write(reg, data));
}

static void switch_adjust_link1(struct net_device *dev)
{
	struct switch_enet_private *priv = netdev_priv(dev);
	struct phy_device *phydev1 = priv->phydev[0];
	int new_state = 0;

	if (phydev1->link != PHY_DOWN) {
		if (phydev1->duplex != priv->phy1_duplex) {
			new_state = 1;
			priv->phy1_duplex = phydev1->duplex;
		}

		if (phydev1->speed != priv->phy1_speed) {
			new_state = 1;
			priv->phy1_speed = phydev1->speed;
		}

		if (priv->phy1_old_link == PHY_DOWN) {
			new_state = 1;
			priv->phy1_old_link = phydev1->link;
		}
	} else if (priv->phy1_old_link) {
		new_state = 1;
		priv->phy1_old_link = PHY_DOWN;
		priv->phy1_speed = 0;
		priv->phy1_duplex = -1;
	}

	if (new_state) {
		ports_link_status.port1_link_status = phydev1->link;
		if (phydev1->link == PHY_DOWN)
			esw_atable_dynamicms_del_entries_for_port(priv, 1);

		/*Send the new status to user space*/
		if (user_pid != 1)
			sys_tkill(user_pid, SIGUSR1);
	}
}

static void switch_adjust_link2(struct net_device *dev)
{
	struct switch_enet_private *priv = netdev_priv(dev);
	struct phy_device *phydev2 = priv->phydev[1];
	int new_state = 0;

	if (phydev2->link != PHY_DOWN) {
		if (phydev2->duplex != priv->phy2_duplex) {
			new_state = 1;
			priv->phy2_duplex = phydev2->duplex;
		}

		if (phydev2->speed != priv->phy2_speed) {
			new_state = 1;
			priv->phy2_speed = phydev2->speed;
		}

		if (priv->phy2_old_link == PHY_DOWN) {
			new_state = 1;
			priv->phy2_old_link = phydev2->link;
		}
	} else if (priv->phy2_old_link) {
		new_state = 1;
		priv->phy2_old_link = PHY_DOWN;
		priv->phy2_speed = 0;
		priv->phy2_duplex = -1;
	}

	if (new_state) {
		ports_link_status.port2_link_status = phydev2->link;
		if (phydev2->link == PHY_DOWN)
			esw_atable_dynamicms_del_entries_for_port(priv, 2);

		/*Send the new status to user space*/
		if (user_pid != 1)
			sys_tkill(user_pid, SIGUSR1);
	}
}

static int coldfire_switch_init_phy(struct net_device *dev)
{
	struct switch_enet_private *priv = netdev_priv(dev);
	struct phy_device *phydev[SWITCH_EPORT_NUMBER] = {NULL, NULL};
	int i = 0;
#ifdef CONFIG_FEC_SHARED_PHY
	int startnode = 0;
#endif
    int mdio_idx = 0;
    int phy_dev_idx = 0;
  
    for(mdio_idx = 0; mdio_idx < MDIO_MAX_BUS; mdio_idx++)
    {
        /* search for connect PHY device */
        for (i = 0; i < PHY_MAX_ADDR; i++) {
            struct phy_device *const tmp_phydev =
                mdiobus_get_phy(priv->mdio_bus[mdio_idx], i);

            if (!tmp_phydev)
                continue;

    #ifdef CONFIG_FEC_SHARED_PHY
            if (priv->index == 0)
                phydev[i] = tmp_phydev;
            else if (priv->index == 1) {
                if (startnode == 1) {
                    phydev[i] = tmp_phydev;
                    startnode = 0;
                } else {
                    startnode++;
                    continue;
                }
            } else
                printk(KERN_INFO "%s now we do not"
                    "support (%d) more than"
                    "2 phys shared "
                    "one mdio bus\n",
                    __func__, startnode);
    #else
            printk(KERN_INFO "Modelo switch: phydev %d, mdio %d, phy %d\n", phy_dev_idx, mdio_idx, i);
            phydev[phy_dev_idx] = tmp_phydev;
            phy_dev_idx++;
    #endif
        }
    }

	/* now we are supposed to have a proper phydev, to attach to... */
	if ((!phydev[0]) && (!phydev[1])) {
		printk(KERN_INFO "%s: Don't found any phy device at all\n",
			dev->name);
		return -ENODEV;
	}

	priv->phy1_link = PHY_DOWN;
	priv->phy1_old_link = PHY_DOWN;
	priv->phy1_speed = 0;
	priv->phy1_duplex = -1;

	priv->phy2_link = PHY_DOWN;
	priv->phy2_old_link = PHY_DOWN;
	priv->phy2_speed = 0;
	priv->phy2_duplex = -1;

	phydev[0] = phy_connect(dev, phydev_name(phydev[0]),
		&switch_adjust_link1, PHY_INTERFACE_MODE_MII);
	if (IS_ERR(phydev[0])) {
		printk(KERN_ERR " %s phy_connect failed\n", __func__);
		return PTR_ERR(phydev[0]);
	}

	phydev[1] = phy_connect(dev, phydev_name(phydev[1]),
		&switch_adjust_link2, PHY_INTERFACE_MODE_MII);
	if (IS_ERR(phydev[1])) {
		printk(KERN_ERR " %s phy_connect failed\n", __func__);
		return PTR_ERR(phydev[1]);
	}

	for (i = 0; i < SWITCH_EPORT_NUMBER; i++) {
		printk(KERN_INFO "attached phy %i to driver %s\n",
			phydev[i]->mdio.addr, phydev[i]->drv->name);
		priv->phydev[i] = phydev[i];
	}

	return 0;
}
/* -----------------------------------------------------------------------*/
static int switch_enet_open(struct net_device *dev)
{
	struct switch_enet_private *fep = netdev_priv(dev);
	volatile switch_t *fecp;
	int i;

	fecp = (volatile switch_t *)fep->hwp;
	/* I should reset the ring buffers here, but I don't yet know
	 * a simple way to do that.
	 */
	switch_set_mac_address(dev);

	fep->phy1_link = 0;
	fep->phy2_link = 0;

	coldfire_switch_init_phy(dev);
	for (i = 0; i < SWITCH_EPORT_NUMBER; i++) {
		phy_write(fep->phydev[i], MII_BMCR, BMCR_RESET);
		phy_start(fep->phydev[i]);
	}

	fep->phy1_old_link = 0;
	fep->phy2_old_link = 0;
	fep->phy1_link = 1;
	fep->phy2_link = 1;

	/* no phy,  go full duplex,  it's most likely a hub chip */
	switch_restart(dev, 1);

	/* if the fec is the fist open, we need to do nothing*/
	/* if the fec is not the fist open, we need to restart the FEC*/
	if (fep->sequence_done == 0)
		switch_restart(dev, 1);
	else
		fep->sequence_done = 0;

	fep->currTime = 0;
	fep->learning_irqhandle_enable = 0;

	MCF_ESW_PER = 0x70007;
	fecp->ESW_DBCR = MCF_ESW_DBCR_P0 | MCF_ESW_DBCR_P1 | MCF_ESW_DBCR_P2;
	fecp->ESW_DMCR = MCF_ESW_DMCR_P0 | MCF_ESW_DMCR_P1 | MCF_ESW_DMCR_P2;

	netif_start_queue(dev);
	fep->opened = 1;

	return 0;
}

static int switch_enet_close(struct net_device *dev)
{
	struct switch_enet_private *fep = netdev_priv(dev);
	int i;

	/* Don't know what to do yet.*/
	fep->opened = 0;
	netif_stop_queue(dev);
	switch_stop(dev);

	for (i = 0; i < SWITCH_EPORT_NUMBER; i++) {
		phy_disconnect(fep->phydev[i]);
		phy_stop(fep->phydev[i]);
		phy_write(fep->phydev[i], MII_BMCR, BMCR_PDOWN);
	}

	return 0;
}

/* Set or clear the multicast filter for this adaptor.
 * Skeleton taken from sunlance driver.
 * The CPM Ethernet implementation allows Multicast as well as individual
 * MAC address filtering.  Some of the drivers check to make sure it is
 * a group multicast address, and discard those that are not.  I guess I
 * will do the same for now, but just remove the test if you want
 * individual filtering as well (do the upper net layers want or support
 * this kind of feature?).
 */

#define CRC32_POLY	0xEDB88320

static void set_multicast_list(struct net_device *dev)
{
	struct switch_enet_private *fep;
	volatile switch_t *ep;
	unsigned int i, bit, data, crc;
	struct netdev_hw_addr *ha;

	fep = netdev_priv(dev);
	ep = fep->hwp;

	if (dev->flags & IFF_PROMISC) {
		printk(KERN_INFO "%s IFF_PROMISC\n", __func__);
	} else {
		if (dev->flags & IFF_ALLMULTI)
			/* Catch all multicast addresses, so set the
			 * filter to all 1's.
			 */
			printk(KERN_INFO "%s IFF_ALLMULTI\n", __func__);
		else {
			netdev_for_each_mc_addr(ha, dev) {
				if (!(ha->addr[0] & 1))
					continue;

				/* calculate crc32 value of mac address
				*/
				crc = 0xffffffff;

				for (i = 0; i < dev->addr_len; i++) {
					data = ha->addr[i];
					for (bit = 0; bit < 8; bit++,
						data >>= 1) {
						crc = (crc >> 1) ^
						(((crc ^ data) & 1) ?
						CRC32_POLY : 0);
					}
				}

			}
		}
	}
}

/* Set a MAC change in hardware.*/
static void switch_set_mac_address(struct net_device *dev)
{
	volatile switch_t *fecp;

	fecp = ((struct switch_enet_private *)netdev_priv(dev))->hwp;
}

static void switch_hw_init(void)
{
	/* GPIO config - RMII mode for both MACs */
	/* MCFGPIO_PAR_FEC = (MCFGPIO_PAR_FEC &
		MCFGPIO_PAR_FEC_FEC_MASK) |
		MCFGPIO_PAR_FEC_FEC_RMII0FUL_1FUL; */
	
	__raw_writeb((__raw_readb(MCFGPIO_PAR_FEC) & MCFGPIO_PAR_FEC_FEC_MASK) | MCFGPIO_PAR_FEC_FEC_RMII0FUL_1FUL,
		MCFGPIO_PAR_FEC);

	/* Initialize MAC 0/1 */
	/* RCR */
	MCF_FEC_RCR0 = (MCF_FEC_RCR_PROM | MCF_FEC_RCR_RMII_MODE |
			MCF_FEC_RCR_MAX_FL(1522) | MCF_FEC_RCR_CRC_FWD);
	MCF_FEC_RCR1 = (MCF_FEC_RCR_PROM | MCF_FEC_RCR_RMII_MODE |
			MCF_FEC_RCR_MAX_FL(1522) | MCF_FEC_RCR_CRC_FWD);
	/* TCR */
	MCF_FEC_TCR0 = MCF_FEC_TCR_FDEN;
	MCF_FEC_TCR1 = MCF_FEC_TCR_FDEN;
	/* ECR */
#ifdef MODELO_BUFFER
	MCF_FEC_ECR0 = MCF_FEC_ECR_ETHER_EN | MCF_FEC_ECR_ENA_1588;
	MCF_FEC_ECR1 = MCF_FEC_ECR_ETHER_EN | MCF_FEC_ECR_ENA_1588;
#else
	MCF_FEC_ECR0 = MCF_FEC_ECR_ETHER_EN;
	MCF_FEC_ECR1 = MCF_FEC_ECR_ETHER_EN;
#endif
	MCF_FEC_MSCR0 = ((((MCF_CLK / 2) / (2500000 / 10)) + 5) / 10) * 2;
	MCF_FEC_MSCR1 = ((((MCF_CLK / 2) / (2500000 / 10)) + 5) / 10) * 2;

	MCF_FEC_EIMR0 = FEC_ENET_TXF | FEC_ENET_RXF;
	MCF_FEC_EIMR1 = FEC_ENET_TXF | FEC_ENET_RXF;
	/*MCF_PPMHR0*/
	MCF_PPMCR0 = 0;
}

static const struct net_device_ops switch_netdev_ops = {
	.ndo_open		= switch_enet_open,
	.ndo_stop		= switch_enet_close,
	.ndo_start_xmit		= switch_enet_start_xmit,
	.ndo_set_rx_mode	= set_multicast_list,
	.ndo_do_ioctl		= switch_enet_ioctl,
	.ndo_tx_timeout		= switch_timeout,
};

/* Initialize the FEC Ethernet.
 */
 /*
  * XXX:  We need to clean up on failure exits here.
  */
static int switch_enet_init(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct switch_enet_private *fep = netdev_priv(dev);
	unsigned long mem_addr;
	cbd_t *bdp;
	cbd_t *cbd_base;
	volatile switch_t *fecp;
	int i, j;
	struct coldfire_switch_platform_data *plat =
		pdev->dev.platform_data;

	/* Allocate memory for buffer descriptors.
	*/
	mem_addr = __get_free_page(GFP_DMA);
	if (mem_addr == 0) {
		printk(KERN_ERR "Switch: allocate descriptor memory failed?\n");
		return -ENOMEM;
	}

	spin_lock_init(&fep->hw_lock);
	spin_lock_init(&fep->mii_lock);

	/* Create an Ethernet device instance.
	*/
	fecp = (volatile switch_t *)plat->switch_hw[0];
	fep->hwp = fecp;
	fep->netdev = dev;

	/*
	 * SWITCH CONFIGURATION
	 */
	fecp->ESW_MODE = MCF_ESW_MODE_SW_RST;
	udelay(10);
	/* enable switch*/
	fecp->ESW_MODE = MCF_ESW_MODE_STATRST;
	fecp->ESW_MODE = MCF_ESW_MODE_SW_EN;

	/* Enable transmit/receive on all ports */
	fecp->ESW_PER = 0xffffffff;

	/* Management port configuration,
	 * make port 0 as management port */
	fecp->ESW_BMPC = 0;

	/* clear all switch irq*/
	fecp->switch_ievent = 0xffffffff;
	fecp->switch_imask  = 0;

	udelay(10);

	/* Set the Ethernet address.  If using multiple Enets on the 8xx,
	 * this needs some work to get unique addresses.
	 *
	 * This is our default MAC address unless the user changes
	 * it via eth_mac_addr (our dev->set_mac_addr handler).
	 */
	if (plat && plat->get_mac)
		plat->get_mac(dev);

	cbd_base = (cbd_t *)mem_addr;
	/* XXX: missing check for allocation failure */
	if (plat && plat->uncache)
		plat->uncache(mem_addr);

	/* Set receive and transmit descriptor base.
	*/
	fep->rx_bd_base = cbd_base;
	fep->tx_bd_base = cbd_base + RX_RING_SIZE;

	dev->base_addr = (unsigned long)fecp;

	/* The FEC Ethernet specific entries in the device structure. */
	dev->watchdog_timeo = TX_TIMEOUT;
	dev->netdev_ops	= &switch_netdev_ops;

	fep->dirty_tx = fep->cur_tx = fep->tx_bd_base;
	fep->cur_rx = fep->rx_bd_base;

	fep->skb_cur = fep->skb_dirty = 0;

	/* Initialize the receive buffer descriptors. */
	bdp = fep->rx_bd_base;

	for (i = 0; i < SWITCH_ENET_RX_PAGES; i++) {

		/* Allocate a page.
		*/
		mem_addr = __get_free_page(GFP_DMA);
		/* XXX: missing check for allocation failure */
		if (plat && plat->uncache)
			plat->uncache(mem_addr);

		/* Initialize the BD for every fragment in the page.
		*/
		for (j = 0; j < SWITCH_ENET_RX_FRPPG; j++) {
			bdp->cbd_sc = BD_ENET_RX_EMPTY;
			bdp->cbd_bufaddr = __pa(mem_addr);
#ifdef MODELO_BUFFER
			bdp->bdu = 0x00000000;
			bdp->ebd_status = RX_BD_INT;
#endif
			mem_addr += SWITCH_ENET_RX_FRSIZE;
			bdp++;
		}
	}

	/* Set the last buffer to wrap.
	*/
	bdp--;
	bdp->cbd_sc |= BD_SC_WRAP;

	/* ...and the same for transmmit.
	*/
	bdp = fep->tx_bd_base;
	for (i = 0, j = SWITCH_ENET_TX_FRPPG; i < TX_RING_SIZE; i++) {
		if (j >= SWITCH_ENET_TX_FRPPG) {
			mem_addr = __get_free_page(GFP_DMA);
			j = 1;
		} else {
			mem_addr += SWITCH_ENET_TX_FRSIZE;
			j++;
		}
		fep->tx_bounce[i] = (unsigned char *) mem_addr;

		/* Initialize the BD for every fragment in the page.
		*/
		bdp->cbd_sc = 0;
		bdp->cbd_bufaddr = 0;
		bdp++;
	}

	/* Set the last buffer to wrap.
	*/
	bdp--;
	bdp->cbd_sc |= BD_SC_WRAP;

	/* Set receive and transmit descriptor base.
	*/
	fecp->fec_r_des_start = __pa((uint)(fep->rx_bd_base));
	fecp->fec_x_des_start = __pa((uint)(fep->tx_bd_base));

	/* Install our interrupt handlers. This varies depending on
	 * the architecture.
	*/
	if (plat && plat->request_intrs)
		plat->request_intrs(dev, switch_enet_interrupt, dev);

	fecp->fec_r_buff_size = RX_BUFFER_SIZE;
	fecp->fec_r_des_active = MCF_ESW_RDAR_R_DES_ACTIVE;

	/* setup MII interface */
	if (plat && plat->set_mii)
		plat->set_mii(dev);

	/* Clear and enable interrupts */
	fecp->switch_ievent = 0xffffffff;
	fecp->switch_imask  = MCF_ESW_IMR_RXB | MCF_ESW_IMR_TXB |
		MCF_ESW_IMR_RXF | MCF_ESW_IMR_TXF;
	esw_clear_atable(fep);
	/* Queue up command to detect the PHY and initialize the
	 * remainder of the interface.
	 */
#ifndef CONFIG_FEC_SHARED_PHY
	fep->phy_addr = 0;
#else
	fep->phy_addr = fep->index;
#endif

	fep->sequence_done = 1;
	return 0;
}

/* This function is called to start or restart the FEC during a link
 * change.  This only happens when switching between half and full
 * duplex.
 */
static void switch_restart(struct net_device *dev, int duplex)
{
	struct switch_enet_private *fep;
	cbd_t *bdp;
	volatile switch_t *fecp;
	int i;
	struct coldfire_switch_platform_data *plat;

	fep = netdev_priv(dev);
	fecp = fep->hwp;
	plat = fep->pdev->dev.platform_data;
	/* Whack a reset.  We should wait for this.*/
	MCF_FEC_ECR0 = 1;
	MCF_FEC_ECR1 = 1;
	udelay(10);

	fecp->ESW_MODE = MCF_ESW_MODE_SW_RST;
	udelay(10);
	fecp->ESW_MODE = MCF_ESW_MODE_STATRST;
	fecp->ESW_MODE = MCF_ESW_MODE_SW_EN;

	/* Enable transmit/receive on all ports */
	fecp->ESW_PER = 0xffffffff;

	/* Management port configuration,
	 * make port 0 as management port */
	fecp->ESW_BMPC = 0;

	/* Clear any outstanding interrupt.
	*/
	fecp->switch_ievent = 0xffffffff;

	/* Set station address.*/
	switch_set_mac_address(dev);

	switch_hw_init();

	/* Reset all multicast.*/

	/* Set maximum receive buffer size.
	*/
	fecp->fec_r_buff_size = PKT_MAXBLR_SIZE;

	if (plat && plat->localhw_setup)
		plat->localhw_setup();
	/* Set receive and transmit descriptor base.
	*/
	fecp->fec_r_des_start = __pa((uint)(fep->rx_bd_base));
	fecp->fec_x_des_start = __pa((uint)(fep->tx_bd_base));

	fep->dirty_tx = fep->cur_tx = fep->tx_bd_base;
	fep->cur_rx = fep->rx_bd_base;

	/* Reset SKB transmit buffers.
	*/
	fep->skb_cur = fep->skb_dirty = 0;
	for (i = 0; i <= TX_RING_MOD_MASK; i++) {
		if (fep->tx_skbuff[i] != NULL) {
			dev_kfree_skb_any(fep->tx_skbuff[i]);
			fep->tx_skbuff[i] = NULL;
		}
	}

	/* Initialize the receive buffer descriptors.
	*/
	bdp = fep->rx_bd_base;
	for (i = 0; i < RX_RING_SIZE; i++) {

		/* Initialize the BD for every fragment in the page.
		*/
		bdp->cbd_sc = BD_ENET_RX_EMPTY;
#ifdef MODELO_BUFFER
		bdp->bdu = 0x00000000;
		bdp->ebd_status = RX_BD_INT;
#endif
		bdp++;
	}

	/* Set the last buffer to wrap.
	*/
	bdp--;
	bdp->cbd_sc |= BD_SC_WRAP;

	/* ...and the same for transmmit.
	*/
	bdp = fep->tx_bd_base;
	for (i = 0; i < TX_RING_SIZE; i++) {

		/* Initialize the BD for every fragment in the page.*/
		bdp->cbd_sc = 0;
		bdp->cbd_bufaddr = 0;
		bdp++;
	}

	/* Set the last buffer to wrap.*/
	bdp--;
	bdp->cbd_sc |= BD_SC_WRAP;

	fep->full_duplex = duplex;

	/* And last, enable the transmit and receive processing.*/
	fecp->fec_r_buff_size = RX_BUFFER_SIZE;
	fecp->fec_r_des_active = MCF_ESW_RDAR_R_DES_ACTIVE;

	/* Enable interrupts we wish to service.
	*/
	fecp->switch_ievent = 0xffffffff;
	fecp->switch_imask  = MCF_ESW_IMR_RXF | MCF_ESW_IMR_TXF |
		MCF_ESW_IMR_RXB | MCF_ESW_IMR_TXB;
}

static void switch_stop(struct net_device *dev)
{
	volatile switch_t *fecp;
	struct switch_enet_private *fep;
	struct coldfire_switch_platform_data *plat;

	fep = netdev_priv(dev);
	fecp = fep->hwp;
	plat = fep->pdev->dev.platform_data;
	/*
	** We cannot expect a graceful transmit stop without link !!!
	*/
	if (fep->phy1_link)
		udelay(10);
	if (fep->phy2_link)
		udelay(10);

	/* Whack a reset.  We should wait for this.
	*/
	udelay(10);
}

static int fec_mdio_register(struct net_device *dev, int bus_number)
{
	int err = 0;
	struct switch_enet_private *fep = netdev_priv(dev);
    char tmp[64];

	fep->mdio_bus[bus_number] = mdiobus_alloc();
	if (!fep->mdio_bus[bus_number]) {
		printk(KERN_ERR "ethernet switch mdiobus_alloc fail\n");
		return -ENOMEM;
	}

    sprintf(tmp, "Coldfire switch MII %d Bus", bus_number);
	fep->mdio_bus[bus_number]->name = tmp;
	sprintf(fep->mdio_bus[bus_number]->id, "%d", bus_number);

	fep->mdio_bus[bus_number]->read = &coldfire_fec_mdio_read;
	fep->mdio_bus[bus_number]->write = &coldfire_fec_mdio_write;
    
	fep->mdio_bus[bus_number]->priv = dev;
	fep->mdio_bus[bus_number]->phy_mask = 0;
	err = mdiobus_register(fep->mdio_bus[bus_number]);
	if (err) {
		mdiobus_free(fep->mdio_bus[bus_number]);
		printk(KERN_ERR "%s: ethernet mdiobus_register fail\n",
			dev->name);
		return -EIO;
	}

	printk(KERN_INFO "mdiobus_register %s ok\n",
		fep->mdio_bus[bus_number]->name);
	return err;
}

static int eth_switch_probe_finish(struct platform_device *pdev)
{
	struct net_device *dev;
	int err;
	struct switch_enet_private *fep;
	struct task_struct *task;

	printk(KERN_INFO "Ethernet Switch Version 1.0\n");

	dev = alloc_etherdev(sizeof(struct switch_enet_private));
	if (!dev) {
		printk(KERN_ERR "%s: ethernet switch alloc_etherdev fail\n",
				dev->name);
		return -ENOMEM;
	}

	SET_NETDEV_DEV(dev, &pdev->dev);

	fep = netdev_priv(dev);

	fep->pdev = pdev;
	platform_set_drvdata(pdev, dev);
	err = switch_enet_init(pdev);
	if (err) {
		free_netdev(dev);
		platform_set_drvdata(pdev, NULL);
	}

	err = fec_mdio_register(dev, 0);
	if (err) {
		printk(KERN_ERR "%s: ethernet switch fec_mdio_register 0\n",
				dev->name);
		free_netdev(dev);
		platform_set_drvdata(pdev, NULL);
		return -ENOMEM;
	}
    
	err = fec_mdio_register(dev, 1);
	if (err) {
		printk(KERN_ERR "%s: ethernet switch fec_mdio_register 1\n",
				dev->name);
		free_netdev(dev);
		platform_set_drvdata(pdev, NULL);
		return -ENOMEM;
	}

	/* setup timer for Learning Aging function */
	timer_setup(&fep->timer_aging, l2switch_aging_timer, 0);
	mod_timer(&fep->timer_aging, jiffies + LEARNING_AGING_TIMER);

	/* register network device*/
	if (register_netdev(dev) != 0) {
		/* XXX: missing cleanup here */
		free_netdev(dev);
		platform_set_drvdata(pdev, NULL);
		printk(KERN_ERR "%s: ethernet switch register_netdev fail\n",
				dev->name);
		return -EIO;
	}

	task = kthread_run(switch_enet_learning, fep,
			"modelo l2switch");
	if (IS_ERR(task)) {
		err = PTR_ERR(task);
		return err;
	}

	printk(KERN_INFO "%s: ethernet switch %pM\n",
			dev->name, dev->dev_addr);
	return 0;
}

static void eth_switch_enabled_work(struct work_struct *work)
{
	if (switch_enabled == 0) {
		/* Not configured, get back later */
		schedule_delayed_work(&config_workqueue, SWE_POLL_TIMING);
	} else if (switch_enabled == 1) {
		/* Switch is enabled, cancel the task and finish probing */
		eth_switch_probe_finish(pdev_copy);
	} else {
		/* An error occured, just stop there */
	}
}

static int eth_switch_probe(struct platform_device *pdev)
{
	/* Save the platform reference */
	pdev_copy = pdev;

	INIT_DELAYED_WORK(&config_workqueue, eth_switch_enabled_work);
	schedule_delayed_work(&config_workqueue, SWE_POLL_TIMING);

	return 0;
}

static void eth_switch_remove(struct platform_device *pdev)
{
	int i;
	struct net_device *dev;
	struct switch_enet_private *fep;
	struct switch_platform_private *chip;

	chip = platform_get_drvdata(pdev);
	if (chip) {
		for (i = 0; i < chip->num_slots; i++) {
			fep = chip->fep_host[i];
			dev = fep->netdev;
			fep->sequence_done = 1;
			unregister_netdev(dev);
			free_netdev(dev);

			timer_delete_sync(&fep->timer_aging);
		}

		platform_set_drvdata(pdev, NULL);
		kfree(chip);

	} else
		printk(KERN_ERR "%s: can not get the "
			"switch_platform_private %x\n", __func__,
			(unsigned int)chip);
}

static struct platform_driver eth_switch_driver = {
	.probe          = eth_switch_probe,
	.remove         = eth_switch_remove,
	.driver         = {
		.name   = "coldfire-switch",
		.owner  = THIS_MODULE,
	},
};

static int coldfire_switch_init(void)
{
	struct dentry *top;

	top = debugfs_lookup("switch_config", NULL);
	if (top == NULL)
		printk(KERN_ERR "Debugfs failed !\n");
	else
		debugfs_create_u8("switch_enabled", S_IRUGO | S_IWUSR, top,
				  &switch_enabled);

	return platform_driver_register(&eth_switch_driver);
}

static void coldfire_switch_exit(void)
{
	platform_driver_unregister(&eth_switch_driver);
}

module_init(coldfire_switch_init);
module_exit(coldfire_switch_exit);
MODULE_LICENSE("GPL");
