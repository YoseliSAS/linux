// SPDX-License-Identifier: GPL-2.0
/*
 * mcf_dma_timer.c -- Freescale ColdFire DMA Timer.
 *
 * Copyright (C) 2024, Jean-Michel Hautbois <jeanmichel.hautbois@yoseli.org>
 *
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/sched_clock.h>

/* DMA timer register offsets from the module base. */
#define DTMR	0x0	/* Mode register (16-bit) */
#define DTXMR	0x2	/* Extended mode register (8-bit) */
#define DTER	0x3	/* Event register (8-bit) */
#define DTRR	0x4	/* Reference register (32-bit) */
#define DTCN	0xc	/* Counter register (32-bit) */

/* DTMR bits. */
#define DMA_DTMR_RST		BIT(0)	/* Enable the timer */
#define DMA_DTMR_CLK_DIV_16	BIT(2)	/* CLK field (bits 2:1) = 0b10: bus clock / 16 */
#define DMA_DTMR_ORRI		BIT(4)	/* Enable interrupt on reference match */

/* DTER bits (write 1 to clear). */
#define DMA_DTER_CAP		BIT(0)	/* Capture event */
#define DMA_DTER_REF		BIT(1)	/* Reference event */

struct dmatmr_priv {
	void __iomem *base;
	struct platform_device *pdev;
	unsigned long rate;
	struct clock_event_device ced;
	struct clocksource cs;
};

static void __iomem *dmatmr_sched_clk_counter;

static struct dmatmr_priv *ced_to_priv(struct clock_event_device *ced)
{
	return container_of(ced, struct dmatmr_priv, ced);
}

static void sys_dtim_init(struct dmatmr_priv *priv)
{
	/*
	 * Start the counter free-running off the internal bus clock divided
	 * by 16. The reference-match interrupt is left disabled here and only
	 * enabled once the clockevent is switched to oneshot mode (see
	 * cfv4_set_oneshot()), so no event can fire before the clockevent is
	 * registered.
	 */
	mcf_write16(DMA_DTMR_CLK_DIV_16 | DMA_DTMR_RST, priv->base + DTMR);
	mcf_write8(0, priv->base + DTXMR);
	mcf_write8(DMA_DTER_REF | DMA_DTER_CAP, priv->base + DTER);
}

static u64 notrace sys_dtim_read(void)
{
	return mcf_read32(dmatmr_sched_clk_counter);
}

static u64 cfv4_read_dtimvalue(struct clocksource *cs)
{
	struct dmatmr_priv *priv = container_of(cs, struct dmatmr_priv, cs);

	return mcf_read32(priv->base + DTCN);
}

static int cfv4_set_next_event(unsigned long delta,
			       struct clock_event_device *dev)
{
	struct dmatmr_priv *priv = ced_to_priv(dev);
	u32 now = mcf_read32(priv->base + DTCN);
	u32 next = now + delta;

	/*
	 * Arm an absolute reference-match on the free-running counter. Called
	 * with local interrupts disabled by the clockevent core, so the read
	 * and the reference update are not preempted by the timer interrupt.
	 */
	mcf_write32(next, priv->base + DTRR);

	/*
	 * The compare only fires on DTCN == DTRR. If the counter has already
	 * passed the deadline, report -ETIME so the core retries with a larger
	 * delta instead of stalling until the 32-bit counter wraps (~9 min).
	 */
	if ((s32)(next - mcf_read32(priv->base + DTCN)) <= 0)
		return -ETIME;

	return 0;
}

static int cfv4_set_oneshot(struct clock_event_device *dev)
{
	struct dmatmr_priv *priv = ced_to_priv(dev);

	/* Clear stale events and enable the reference-match interrupt. */
	mcf_write8(DMA_DTER_REF | DMA_DTER_CAP, priv->base + DTER);
	mcf_write16(DMA_DTMR_ORRI | DMA_DTMR_CLK_DIV_16 | DMA_DTMR_RST,
		     priv->base + DTMR);

	return 0;
}

static int cfv4_set_shutdown(struct clock_event_device *dev)
{
	struct dmatmr_priv *priv = ced_to_priv(dev);

	/*
	 * Disable the reference-match interrupt but keep the counter running:
	 * it also backs the clocksource and sched_clock.
	 */
	mcf_write16(DMA_DTMR_CLK_DIV_16 | DMA_DTMR_RST, priv->base + DTMR);
	mcf_write8(DMA_DTER_REF | DMA_DTER_CAP, priv->base + DTER);

	return 0;
}

static irqreturn_t coldfire_dtim_clk_irq(int irq, void *dev)
{
	struct dmatmr_priv *priv = dev;

	/* Acknowledge the event so the module can re-arm. */
	mcf_write8(DMA_DTER_REF | DMA_DTER_CAP, priv->base + DTER);

	priv->ced.event_handler(&priv->ced);

	return IRQ_HANDLED;
}

static int mcf_dma_register_clocksource(struct dmatmr_priv *priv)
{
	struct clocksource *cs = &priv->cs;

	cs->name = dev_name(&priv->pdev->dev);
	cs->rating = 250;
	cs->mask = CLOCKSOURCE_MASK(32);
	cs->read = cfv4_read_dtimvalue;
	cs->flags = CLOCK_SOURCE_IS_CONTINUOUS;

	return clocksource_register_hz(cs, priv->rate);
}

static void mcf_dma_register_clockevent(struct dmatmr_priv *priv)
{
	struct clock_event_device *ced = &priv->ced;

	ced->name = dev_name(&priv->pdev->dev);
	ced->features = CLOCK_EVT_FEAT_ONESHOT;
	ced->rating = 250;
	ced->cpumask = cpumask_of(0);
	ced->set_state_oneshot = cfv4_set_oneshot;
	ced->set_state_shutdown = cfv4_set_shutdown;
	ced->set_next_event = cfv4_set_next_event;

	clockevents_config_and_register(ced, priv->rate, 2, 0xfffffffe);
}

static int __init mcf_dma_timer_probe(struct platform_device *pdev)
{
	struct dmatmr_priv *priv;
	struct clk *clk;
	int irq, ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
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

	/*
	 * Disable the timer and clear any event a bootloader or a previous
	 * kernel (warm reboot, kexec) may have left armed, before the interrupt
	 * line is unmasked by request_irq().
	 */
	mcf_write16(0, priv->base + DTMR);
	mcf_write8(DMA_DTER_REF | DMA_DTER_CAP, priv->base + DTER);

	ret = devm_request_irq(&pdev->dev, irq, coldfire_dtim_clk_irq, IRQF_TIMER,
			       dev_name(&pdev->dev), priv);
	if (ret) {
		dev_err(&pdev->dev, "failed to request irq %d\n", irq);
		return ret;
	}

	clk = devm_clk_get_enabled(&pdev->dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(clk),
				     "failed to get clock\n");

	priv->rate = clk_get_rate(clk) / 16;
	if (!priv->rate)
		return -EINVAL;

	/* Start the free-running counter before exposing it as a timer. */
	sys_dtim_init(priv);

	dmatmr_sched_clk_counter = priv->base + DTCN;
	sched_clock_register(sys_dtim_read, 32, priv->rate);

	ret = mcf_dma_register_clocksource(priv);
	if (ret)
		return ret;

	mcf_dma_register_clockevent(priv);

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
