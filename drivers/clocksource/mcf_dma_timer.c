// SPDX-License-Identifier: GPL-2.0
/*
 * mcf_dma_timer.c -- Freescale ColdFire DMA Timer.
 *
 * Copyright (C) 2024, Jean-Michel Hautbois <jeanmichel.hautbois@yoseli.org>
 *
 */

#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/sched/clock.h>
#include <linux/sched_clock.h>

#include <asm/machdep.h>
#include <asm/coldfire.h>
#include <asm/mcfpit.h>
#include <asm/mcfsim.h>

/*
 * regs->dtmr = priv->base + 0x0;
 * regs->dtxmr = priv->base + 0x2;
 * regs->dter = priv->base + 0x3;
 * regs->dtrr = priv->base + 0x4;
 * regs->dtcr = priv->base + 0x8;
 * regs->dtcn = priv->base + 0xc;
 */
#define DTMR 0x0
#define DTXMR 0x2
#define DTER 0x3
#define DTRR 0x4
#define DTCR 0x8
#define DTCN 0xc

struct dmatmr_priv {
	void __iomem *base;
	struct clk *clk;
	struct platform_device *pdev;
	unsigned long rate;
	struct clock_event_device ced;
	struct clocksource cs;
	int id;
};

static u64 sched_dtim_clk_val;
static void __iomem *dmatmr_sched_clk_counter;

static struct dmatmr_priv *ced_to_priv(struct clock_event_device *ced)
{
	return container_of(ced, struct dmatmr_priv, ced);
}

static void sys_dtim_init(struct dmatmr_priv *priv)
{
	__raw_writel((priv->rate / HZ) - 1, priv->base + DTRR);
	__raw_writew(BIT(4) | BIT(1) | BIT(0), priv->base + DTMR);
	__raw_writeb(0, priv->base + DTXMR);
	__raw_writeb(1, priv->base + DTER);

	dev_info(&priv->pdev->dev, "Initialized for sched_clock at %d hz", HZ);
}

static inline u64 read_dtcn(void)
{
	return __raw_readl(dmatmr_sched_clk_counter);
}

static u64 notrace sys_dtim_read(void)
{
	return READ_ONCE(sched_dtim_clk_val) + read_dtcn();
}

static u64 cfv4_read_dtimvalue(struct clocksource *cs)
{
	u64 val;

	val = sys_dtim_read();

	return val;
}

static int cfv4_set_next_event(unsigned long delta,
	struct clock_event_device *dev)
{
	struct dmatmr_priv *priv = ced_to_priv(dev);

	/* read timer value */
	sched_dtim_clk_val += read_dtcn();

	/* reset timer with delta cycle */
	__raw_writew(0, priv->base + DTMR);
	__raw_writel(delta, priv->base + DTRR);
	__raw_writew(BIT(4) | BIT(1) | BIT(0), priv->base + DTMR);

	return 0;
}

static int cfv4_set_oneshot(struct clock_event_device *dev)
{
	struct dmatmr_priv *priv = ced_to_priv(dev);

	/* read timer value */
	sched_dtim_clk_val += read_dtcn();

	__raw_writew(0, priv->base + DTMR);
	return 0;
}

static irqreturn_t coldfire_dtim_clk_irq(int irq, void *dev)
{
	struct dmatmr_priv *priv = dev;

	/* acknowledge the IRQ */
	__raw_writeb(BIT(0) | BIT(1), priv->base + DTER);

	/* read timer value */
	sched_dtim_clk_val += read_dtcn();

	/* restart counter */
	__raw_writel(0, dmatmr_sched_clk_counter);

	priv->ced.event_handler(&priv->ced);

	return IRQ_HANDLED;
}

static void mcf_dma_register_clocksource(struct dmatmr_priv *priv)
{
	struct clocksource *cs = &priv->cs;

	cs->name = dev_name(&priv->pdev->dev);
	cs->rating = 250;
	cs->mask = CLOCKSOURCE_MASK(30);
	cs->read = cfv4_read_dtimvalue;
	cs->flags = CLOCK_SOURCE_IS_CONTINUOUS;

	dev_info(&priv->pdev->dev, "registering clocksource\n");

	clocksource_register_hz(cs, priv->rate);
}

static void mcf_dma_register_clockevent(struct dmatmr_priv *priv)
{
	struct clock_event_device *ced = &priv->ced;

	ced->name = dev_name(&priv->pdev->dev);
	ced->features = CLOCK_EVT_FEAT_ONESHOT;
	ced->rating = 250;
	ced->shift = 20;
	ced->cpumask = cpumask_of(smp_processor_id());
	ced->set_state_oneshot = cfv4_set_oneshot;
	ced->set_next_event = cfv4_set_next_event;

	dev_info(&priv->pdev->dev, "registering clockevent\n");

	clockevents_config_and_register(ced, priv->rate, 2, 0xffffffff);
}

static int __init mcf_dma_timer_probe(struct platform_device *pdev)
{
	struct dmatmr_priv *priv;
	struct resource *prio_reg;
	int irq, ret;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->pdev = pdev;
	platform_set_drvdata(pdev, priv);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->id = pdev->id;

	ret = devm_request_irq(&pdev->dev, irq, coldfire_dtim_clk_irq, IRQF_TIMER,
			       dev_name(&pdev->dev), priv);
	if (ret) {
		dev_err(&pdev->dev, "failed to request irq %d\n", irq);
		return ret;
	}

	prio_reg = platform_get_resource_byname(pdev, IORESOURCE_REG, "prio_reg");
	if (prio_reg) {
		/* Enhance the interrupt priority (0 == disabled) */
		__raw_writeb(4, prio_reg->start);
		dev_info(&pdev->dev, "Enhance priority to 4\n");
	}

	priv->clk = devm_clk_get_enabled(&pdev->dev, NULL);
	if (IS_ERR(priv->clk)) {
		dev_err(&pdev->dev, "failed to get clock\n");
		return PTR_ERR(priv->clk);
	}

	priv->rate = clk_get_rate(priv->clk);

	dmatmr_sched_clk_counter = priv->base + DTCN;

	mcf_dma_register_clockevent(priv);
	mcf_dma_register_clocksource(priv);

	/* initialize the system timer */
	sys_dtim_init(priv);

	sched_clock_register(sys_dtim_read, 32, priv->rate);

	return 0;
}

static struct platform_driver mcf_platform_driver = {
	.driver		= {
		.name	= "mcftmr",
	},
};

builtin_platform_driver_probe(mcf_platform_driver, mcf_dma_timer_probe);

MODULE_AUTHOR("Jean-Michel Hautbois <jeanmichel.hautbois@yoseli.org>");
MODULE_DESCRIPTION("Freescale ColdFire dma timer driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:mcftmr");
