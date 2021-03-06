/*
 *  linux/arch/arm/mach-k3v2/timer.c
 *
 *  Copyright (C) 1999 - 2003 ARM Limited
 *  Copyright (C) 2000 Deep Blue Solutions Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/err.h>

#include <asm/hardware/arm_timer.h>
#include <asm/mach/time.h>
#include <mach/hisi_cortex-a9.h>
#include <mach/early-debug.h>
/*
 * Notice:
 *  timer01 (2 no power down timers) --->cpu0-timer & free-run timer
 *  timer23 (2 no power down timers) --->not used & not used
 *  timer45 (2 power down timers)    --->cpu1-timer & cpu2-timer
 *  timer67 (2 power down timers)    --->cpu3-timer & not used
 *  timer89 (2 power down timers)    --->mcu using both
 */

/*
 * These timers are currently always setup to be clocked at 26MHz.
 */
#define TIMER_RELOAD (BUSCLK_TO_TIMER_RELOAD(CONFIG_DEFAULT_TIMERCLK))

#define TIME0_ENABLE_BIT       1<<0

static unsigned long timer0_clk_hz = (CONFIG_DEFAULT_TIMERCLK);
static unsigned long timer1_clk_khz = (CONFIG_DEFAULT_TIMERCLK / 1000);

/* free run timer for sched_clock */
static void __iomem *clksrc_base;
static unsigned long long hisi_cycles_2_ns(unsigned long long cyc)
{
	unsigned long cyc2ns_scale = (1000000 << 10) / timer1_clk_khz;
	return (cyc * cyc2ns_scale) >> 10;
}

static struct irqaction freerun_timer_irq;
void __init freerun_timer_init(void __iomem *base, unsigned int timer_irq)
{
	clksrc_base = base;

	/* setup timer 0 as free-running clocksource */
	writel(0, clksrc_base + TIMER_CTRL);
	writel(0xffffffff, clksrc_base + TIMER_LOAD);
	writel(0xffffffff, clksrc_base + TIMER_VALUE);
	writel(0xffffffff, clksrc_base + TIMER_BGLOAD);
	writel(TIMER_CTRL_32BIT | TIMER_CTRL_ENABLE | TIMER_CTRL_PERIODIC |
		TIMER_CTRL_IE, clksrc_base + TIMER_CTRL);

	setup_irq(timer_irq, &freerun_timer_irq);

        clocksource_mmio_init(clksrc_base + TIMER_VALUE, "free-run-timer",
			CONFIG_DEFAULT_TIMERCLK, 200, 32, clocksource_mmio_readl_down);
}

static unsigned long long free_timer_overflows;
/*
 * IRQ handler for the free run timer
 */
static irqreturn_t freerun_timer_interrupt(int irq, void *dev_id)
{
	if (readl(clksrc_base + TIMER_RIS) & 0x1) {
		free_timer_overflows++;

		/* clear the interrupt */
		writel(1, clksrc_base + TIMER_INTCLR);
	}

	return IRQ_HANDLED;
}

static struct irqaction freerun_timer_irq = {
	.name           = "free-run-timer",
	.flags          = IRQF_SHARED | IRQF_TIMER,
	.handler        = freerun_timer_interrupt,
};

/* FIXME: timer IO can be read after system IO mapped */
static unsigned long iomapped = 0;
unsigned long long sched_clock(void)
{
	unsigned long long ticks64;
	unsigned long ticks2, ticks1;

	if (!iomapped)
		return jiffies_64*(1000000000UL/HZ);

	ticks2 = 0UL - (unsigned long) readl(clksrc_base + TIMER_VALUE);

	do {
		ticks1 = ticks2;
		ticks64 = free_timer_overflows;
		ticks2 = 0UL - (unsigned long) readl(clksrc_base + TIMER_VALUE);
	} while (ticks1 > ticks2);

	/*
	** If INT is not cleaned, means the function is called with irq_save.
	** The ticks has overflow but 'free_timer_overflows' is not be update.
	*/
	if (readl(clksrc_base + TIMER_MIS)) {
		ticks64 += 1;
		ticks2 = 0UL - (unsigned long) readl(clksrc_base + TIMER_VALUE);
	}

	return hisi_cycles_2_ns((ticks64 << 32) | ticks2);
}


/* system clock event in cpu0 */
static void __iomem *clkevt_base;
/*
 * IRQ handler for the system timer
 */
static irqreturn_t sp804_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = dev_id;

	if ((readl(clkevt_base + TIMER_RIS)) & 0x1) {
		/* clear the interrupt */
		writel(1, clkevt_base + TIMER_INTCLR);

		evt->event_handler(evt);
	}

	return IRQ_HANDLED;
}

static void sp804_set_mode(enum clock_event_mode mode,
				struct clock_event_device *evt)
{
	unsigned long ctrl = TIMER_CTRL_32BIT | TIMER_CTRL_IE;

	writel(ctrl, clkevt_base + TIMER_CTRL);

	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		writel(TIMER_RELOAD, clkevt_base + TIMER_LOAD);
		ctrl |= TIMER_CTRL_PERIODIC | TIMER_CTRL_ENABLE;
		break;

	case CLOCK_EVT_MODE_ONESHOT:
		/* period set, and timer enabled in 'next_event' hook */
		ctrl |= TIMER_CTRL_ONESHOT;
		break;

	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
	default:
		break;
	}

	writel(ctrl, clkevt_base + TIMER_CTRL);
}

static int sp804_set_next_event(unsigned long next,
	struct clock_event_device *evt)
{
	unsigned long ctrl = readl(clkevt_base + TIMER_CTRL);

	writel(next, clkevt_base + TIMER_LOAD);
	writel(ctrl | TIMER_CTRL_ENABLE, clkevt_base + TIMER_CTRL);

	return 0;
}

static struct clock_event_device sp804_clockevent = {
	.name		= "timer0",
	.shift		= 32,
	.features       = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
	.set_mode	= sp804_set_mode,
	.set_next_event	= sp804_set_next_event,
	.rating		= 300,
};

static struct irqaction sp804_timer_irq = {
	.name		= "cpu0-timer",
	.flags		= IRQF_SHARED | IRQF_DISABLED | IRQF_TIMER,
	.handler	= sp804_timer_interrupt,
	.dev_id		= &sp804_clockevent,
};

void __init sp804_clockevents_init(void __iomem *base, unsigned int timer_irq)
{
	struct clock_event_device *evt = &sp804_clockevent;

	clkevt_base = base;

	setup_irq(timer_irq, &sp804_timer_irq);

	evt->irq = timer_irq;
	evt->mult = div_sc(timer0_clk_hz, NSEC_PER_SEC, evt->shift);
	evt->max_delta_ns = clockevent_delta2ns(0xffffffff, evt);
	evt->min_delta_ns = clockevent_delta2ns(0xf, evt);
	evt->cpumask = cpumask_of(0);
	clockevents_register_device(evt);
}

/* enable timer clk in sctl reg */
static void hisik3_sctl_enable_clk(void)
{
	unsigned long ctrl = readl(IO_ADDRESS(REG_BASE_SCTRL) + 0x028);

#ifdef CONFIG_MACH_TC45MSU3
	/* FIXME: enable Timer0 clock. should enable in fastboot by chanjinn */
	writel(0x01FFF8104, IO_ADDRESS(REG_BASE_SCTRL));
#endif

	ctrl = ctrl|TIME0_ENABLE_BIT;
	writel(ctrl,(IO_ADDRESS(REG_BASE_SCTRL)+0x20));
}

#define hisik3_ticks2us(ticks) (((ticks)*((1000000/HZ) >> 2))/(TIMER_RELOAD >> 2))

static void __init hisik3_timer_init(void)
{
	unsigned long timer_base = IO_ADDRESS(REG_BASE_TIMER0);
	unsigned long ctrl = TIMER_CTRL_32BIT | TIMER_CTRL_IE;

	edb_trace(1);

	hisik3_sctl_enable_clk();

	writel(0, timer_base + TIMER_CTRL);

	sp804_clockevents_init((void *) timer_base, IRQ_TIMER0);

	writel(0, timer_base + TIMER_CTRL);

	writel(ctrl, timer_base + TIMER_CTRL);

	writel(TIMER_RELOAD, timer_base + TIMER_LOAD);

	ctrl |= TIMER_CTRL_PERIODIC | TIMER_CTRL_ENABLE;

	writel(ctrl, timer_base + TIMER_CTRL);

	/* free run timer in TIMER5 */
	free_timer_overflows = 0;

	freerun_timer_init((void *) timer_base + 0x20, IRQ_TIMER1);

	iomapped = 1;

	edb_trace(1);
}

#ifdef CONFIG_ARCH_USES_GETTIMEOFFSET
unsigned long hisik3_gettimeoffset(void)
{
	unsigned long ticks1, ticks2, status;

	ticks2 = readl(clkevt_base + TIMER_VALUE);
	do {
		ticks1 = ticks2;
		status = readl(clkevt_base + TIMER_RIS);
		ticks2 = readl(clkevt_base + TIMER_VALUE);
	} while ((ticks2 > ticks1) || (ticks2 == 0));

	ticks1 = TIMER_RELOAD - ticks2;

	if (status & 0x1)
		ticks1 += TIMER_RELOAD;

	return hisik3_ticks2us(ticks1);
}
#endif

struct sys_timer k3v2_timer = {
	.init = hisik3_timer_init,
#ifdef CONFIG_ARCH_USES_GETTIMEOFFSET
	.offset = hisik3_gettimeoffset,
#endif
};
