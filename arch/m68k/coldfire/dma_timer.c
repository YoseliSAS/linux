// SPDX-License-Identifier: GPL-2.0
/*
 * dma_timer.c -- Freescale ColdFire DMA Timer.
 *
 * Copyright (C) 2007, Benedikt Spranger <b.spranger@linutronix.de>
 * Copyright (C) 2008. Sebastian Siewior, Linutronix
 *
 */

#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/io.h>
#include <linux/sched/clock.h>
#include <linux/sched_clock.h>

#include <asm/machdep.h>
#include <asm/coldfire.h>
#include <asm/mcfpit.h>
#include <asm/mcfsim.h>

#ifndef CONFIG_M5441x
#define DMA_TIMER_0	(0x00)
#define DMA_TIMER_1	(0x40)
#define DMA_TIMER_2	(0x80)
#define DMA_TIMER_3	(0xc0)

#define DTMR	(MCF_IPSBAR + DMA_TIMER_0 + 0x400)
#define DTXMR	(MCF_IPSBAR + DMA_TIMER_0 + 0x402)
#define DTER	(MCF_IPSBAR + DMA_TIMER_0 + 0x403)
#define DTRR	(MCF_IPSBAR + DMA_TIMER_0 + 0x404)
#define DTCR	(MCF_IPSBAR + DMA_TIMER_0 + 0x408)
#define DTCN	(MCF_IPSBAR + DMA_TIMER_0 + 0x40c)
#else
#define DTMR0	(MCFDMATIMER_BASE0 + 0x0)
#define DTXMR0	(MCFDMATIMER_BASE0 + 0x2)
#define DTER0	(MCFDMATIMER_BASE0 + 0x3)
#define DTRR0	(MCFDMATIMER_BASE0 + 0x4)
#define DTCR0	(MCFDMATIMER_BASE0 + 0x8)
#define DTCN0	(MCFDMATIMER_BASE0 + 0xc)

#define DTMR1	(MCFDMATIMER_BASE1 + 0x0)
#define DTXMR1	(MCFDMATIMER_BASE1 + 0x2)
#define DTER1	(MCFDMATIMER_BASE1 + 0x3)
#define DTRR1	(MCFDMATIMER_BASE1 + 0x4)
#define DTCR1	(MCFDMATIMER_BASE1 + 0x8)
#define DTCN1	(MCFDMATIMER_BASE1 + 0xc)

#define DTMR2	(MCFDMATIMER_BASE2 + 0x0)
#define DTXMR2	(MCFDMATIMER_BASE2 + 0x2)
#define DTER2	(MCFDMATIMER_BASE2 + 0x3)
#define DTRR2	(MCFDMATIMER_BASE2 + 0x4)
#define DTCR2	(MCFDMATIMER_BASE2 + 0x8)
#define DTCN2	(MCFDMATIMER_BASE2 + 0xc)

#define DTMR3	(MCFDMATIMER_BASE3 + 0x0)
#define DTXMR3	(MCFDMATIMER_BASE3 + 0x2)
#define DTER3	(MCFDMATIMER_BASE3 + 0x3)
#define DTRR3	(MCFDMATIMER_BASE3 + 0x4)
#define DTCR3	(MCFDMATIMER_BASE3 + 0x8)
#define DTCN3	(MCFDMATIMER_BASE3 + 0xc)
#endif

#define DMA_FREQ    ((MCF_CLK / 2) / 16)

/* DTMR */
#define DMA_DTMR_RESTART	(1 << 3)
#define DMA_DTMR_CLK_DIV_1	(1 << 1)
#define DMA_DTMR_CLK_DIV_16	(2 << 1)
#define DMA_DTMR_ENABLE		(1 << 0)

#ifndef CONFIG_M5441x
static u64 cf_dt_get_cycles(struct clocksource *cs)
{
	return __raw_readl(DTCN);
}

static struct clocksource clocksource_cf_dt = {
	.name		= "coldfire_dma_timer",
	.rating		= 200,
	.read		= cf_dt_get_cycles,
	.mask		= CLOCKSOURCE_MASK(32),
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

static int __init init_cf_dt_clocksource(void)
{
	/*
	 * We setup DMA timer 0 in free run mode. This incrementing counter is
	 * used as a highly precious clock source. With MCF_CLOCK = 150 MHz we
	 * get a ~213 ns resolution and the 32bit register will overflow almost
	 * every 15 minutes.
	 */
	__raw_writeb(0x00, DTXMR);
	__raw_writeb(0x00, DTER);
	__raw_writel(0x00000000, DTRR);
	__raw_writew(DMA_DTMR_CLK_DIV_16 | DMA_DTMR_ENABLE, DTMR);
	return clocksource_register_hz(&clocksource_cf_dt, DMA_FREQ);
}

arch_initcall(init_cf_dt_clocksource);

#define CYC2NS_SCALE_FACTOR 10 /* 2^10, carefully chosen */
#define CYC2NS_SCALE	((1000000 << CYC2NS_SCALE_FACTOR) / (DMA_FREQ / 1000))

static unsigned long long cycles2ns(unsigned long cycl)
{
	return (unsigned long long) ((unsigned long long)cycl *
			CYC2NS_SCALE) >> CYC2NS_SCALE_FACTOR;
}

unsigned long long sched_clock(void)
{
	unsigned long cycl = __raw_readl(DTCN);

	return cycles2ns(cycl);
}
#else
static u64 sched_dtim_clk_val;

static void sys_dtim_init(void)
{
	__raw_writew(0, DTMR0);
	__raw_writew(0, DTMR1);
	__raw_writew(0, DTMR2);
	__raw_writew(0, DTMR3);

	printk("Initializing DTIM2 for sched_clock at %d hz", HZ);
	__raw_writel((MCF_BUSCLK / HZ) - 1, DTRR2);
	__raw_writew(BIT(4) | BIT(1) | BIT(0), DTMR2);
	__raw_writeb(0, DTXMR2);
	__raw_writeb(1, DTER2);

	printk("Initializing DTIM3 for sched_clock\n");
}

/* DTIM2 */
static inline u64 read_dtcn2(void)
{
	return __raw_readl(DTCN2);
}

static u64 notrace sys_dtim2_read(void)
{
	return sched_dtim_clk_val + read_dtcn2();
}

static u64 cfv4_read_dtim2value(struct clocksource *cs)
{
	return sys_dtim2_read();
}

static int cfv4_set_next_event(unsigned long delta,
	struct clock_event_device *dev)
{
	/* read timer value */
	sched_dtim_clk_val += read_dtcn2();

	/* reset timer with delta cycle */
	__raw_writew(0, DTMR2);
	__raw_writel(delta, DTRR2);
	__raw_writew(BIT(4) | BIT(1) | BIT(0), DTMR2);

	return 0;
}

static int cfv4_set_oneshot(struct clock_event_device *dev)
{
	/* read timer value */
	sched_dtim_clk_val += read_dtcn2();

	__raw_writew(0, DTMR2);
	return 0;
}

static irqreturn_t coldfire_dtim_clk_irq(int irq, void *dev)
{
	struct clock_event_device *evt = (struct clock_event_device *)dev;

	/* acknowledge the IRQ */
	__raw_writeb(BIT(0) | BIT(1), DTER2);

	/* read timer value */
	sched_dtim_clk_val += read_dtcn2();

	/* restart counter */
	__raw_writel(0, DTCN2);

	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static struct clock_event_device clockevent_cfv4 = {
	.name			= "CFV4 timer2even",
	.features		= CLOCK_EVT_FEAT_ONESHOT,
	.rating			= 250,
	.shift			= 20,
	.set_state_oneshot 	= cfv4_set_oneshot,
	.set_next_event		= cfv4_set_next_event,
};

struct clocksource clocksource_cfv4 = {
	.name	= "ColdfireV4",
	.rating	= 250,
	.mask	= CLOCKSOURCE_MASK(30),
	.read	= cfv4_read_dtim2value,
	.flags	= CLOCK_SOURCE_IS_CONTINUOUS,
};

void __init hw_timer_init(void)
{
	int ret;

	clockevent_cfv4.cpumask = cpumask_of(smp_processor_id());
	clockevent_cfv4.mult = div_sc(MCF_BUSCLK, NSEC_PER_SEC, clockevent_cfv4.shift);
	clockevent_cfv4.max_delta_ns = clockevent_delta2ns((MCF_BUSCLK / HZ),&clockevent_cfv4);
	clockevent_cfv4.min_delta_ns = clockevent_delta2ns(1, &clockevent_cfv4);

	printk(KERN_INFO "Register CFV4 clockevent\n");
	printk(KERN_INFO "CPU clock: %dHz\n", MCF_CLK);
	printk(KERN_INFO "BUS clock: %dHz\n", MCF_BUSCLK);
	clockevents_register_device(&clockevent_cfv4);

	/* initialize the system timer */
	sys_dtim_init();

	ret = request_irq(MCFDMATIMER_IRQ_DTIM2, coldfire_dtim_clk_irq, IRQF_TIMER, "timer",
			  (void *)&clockevent_cfv4);
	if (ret) {
		pr_err("Failed to request irq %d (timer): %pe\n", MCFDMATIMER_IRQ_DTIM2,
		       ERR_PTR(ret));
	}

	clocksource_register_hz(&clocksource_cfv4, MCF_BUSCLK);

	/* Enhance the interrupt priority */
	__raw_writeb(5, MCFINTC0_ICR0 + MCFINT0_TIMER2);

	sched_clock_register(sys_dtim2_read, 32, MCF_BUSCLK);
}
#endif
