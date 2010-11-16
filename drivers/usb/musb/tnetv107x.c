/*
 * Texas Instruments TNETV107x "glue layer"
 *
 * Author: pgriffin@mpcdata.com
 *
 * Based on Texas Instruments DA8xx/OMAP-L1x "glue layer"
 *
 * Copyright (c) 2008-2009 MontaVista Software, Inc. <source@mvista.com>
 *
 * Based on the DaVinci "glue layer" code.
 * Copyright (C) 2005-2006 by Texas Instruments
 *
 * This file is part of the Inventra Controller Driver for Linux.
 *
 * The Inventra Controller Driver for Linux is free software; you
 * can redistribute it and/or modify it under the terms of the GNU
 * General Public License version 2 as published by the Free Software
 * Foundation.
 *
 * The Inventra Controller Driver for Linux is distributed in
 * the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with The Inventra Controller Driver for Linux ; if not,
 * write to the Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA  02111-1307  USA
 *
 */

#define DEBUG
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/regulator/consumer.h>
#include <linux/sched.h>

#include <mach/tnetv107x.h>
#include <mach/usb.h>

#include <mach/cppi41.h>

#include "musb_core.h"
#include "cppi41_dma.h"




struct tnetv107x_musb_data {
	struct	work_struct vbus_work;
	struct	regulator *vbus_regulator;
	struct	timer_list otg_workaround;
	spinlock_t lock;
	int	is_on;
	int	vbus_state;
};

/* USB 2.0 OTG module registers */
#define USB_REVISION_REG	0x00
#define USB_CTRL_REG		0x04
#define USB_STAT_REG		0x08
#define USB_EMULATION_REG	0x0c
/* 0x10 reserved */
#define USB_AUTOREQ_REG		0x14
#define USB_SRP_FIX_TIME_REG	0x18
#define USB_TEARDOWN_REG	0x1c

#define EP_INTR_SRC_REG		0x20
#define EP_INTR_SRC_SET_REG	0x24
#define EP_INTR_SRC_CLEAR_REG	0x28
#define EP_INTR_MASK_REG	0x2c
#define EP_INTR_MASK_SET_REG	0x30
#define EP_INTR_MASK_CLEAR_REG	0x34
#define EP_INTR_SRC_MASKED_REG	0x38
/* 0x3c reserved */
#define CORE_INTR_SRC_REG	0x40
#define CORE_INTR_SRC_SET_REG	0x44
#define CORE_INTR_SRC_CLEAR_REG	0x48
#define CORE_INTR_MASK_REG	0x4c
#define CORE_INTR_MASK_SET_REG	0x50
#define CORE_INTR_MASK_CLEAR_REG 0x54
#define CORE_INTR_SRC_MASKED_REG 0x58
/* 0x5c reserved */
#define USB_END_OF_INTR_REG	0x60

/*
 * TNETV107X specific definitions
 */

#define TX_EP_MASK	0xffff		/* EP0 + 15 Tx EPs */
#define RX_EP_MASK	0xfffe		/* 15 Rx EPs */

#define USB_INTR_RX_SHIFT	16
#define USB_INTR_TX_SHIFT	0

#define TX_INTR_MASK	(TX_EP_MASK << USB_INTR_TX_SHIFT)
#define RX_INTR_MASK	(RX_EP_MASK << USB_INTR_RX_SHIFT)

#define USB_INTR_USB_SHIFT	16
#define USB_INTR_USB_MASK	(0x1ff << USB_INTR_USB_SHIFT)
#define USB_INTR_DRVVBUS	0x100

/* Control register bits */
#define USB_SOFT_RESET_MASK	1
#define MENTOR_CORE_OFFSET	0x400

/* TNETV107x PHY regs and control register bits */

#define USB_PHY_CTRL		0x08087118
#define USB_PHY_RESET		0x0808a020

/* controller 0 bits 0-6, controller 1 bits 8-14 */
#define VBUSSENS(controller)	(1 << (0 + (controller * 8)))
#define PHYCLKGD(controller)	(1 << (1 + (controller * 8)))
#define VBUSDETEN(controller)	(1 << (2 + (controller * 8)))
#define SENSEEN(controller)	(1 << (3 + (controller * 8)))
#define OTGPWRDN(controller)	(1 << (4 + (controller * 8)))
#define PHYPWRDN(controller)	(1 << (5 + (controller * 8)))
#define PHYPLLON(controller)	(1 << (6 + (controller * 8)))

static DEFINE_SPINLOCK(tnetv107x_phy_lock);

void tnetvevm_deferred_drvvbus(struct work_struct *work)
{
	struct tnetv107x_musb_data *tnetv_bdata =
	container_of(work, struct tnetv107x_musb_data, vbus_work);
	int check, flags, is_on;

	spin_lock_irqsave(&tnetv_bdata->lock, flags);
	is_on = tnetv_bdata->is_on;
	spin_unlock_irqrestore(&tnetv_bdata->lock, flags);

	/* we can't rely on regulator_enabled to determine state, as this
	   doesn't take into consideration ref counting, and the PMIC
	   may have been disabled due to an overcurrent alarm */

	if(!is_on && tnetv_bdata->vbus_state) {
		regulator_disable(tnetv_bdata->vbus_regulator);
		tnetv_bdata->vbus_state = 0;

		check = regulator_is_enabled(tnetv_bdata->vbus_regulator);
		if(check != 0)
			WARNING("regulator disabled, but still on\n");
	}

	if (is_on && !tnetv_bdata->vbus_state) {
		regulator_enable(tnetv_bdata->vbus_regulator);
		tnetv_bdata->vbus_state = 1;

		check = regulator_is_enabled(tnetv_bdata->vbus_regulator);
		if(check != 1)
			WARNING("regulator enabled, but still off\n");
	}

	return;
}

static void tnetv107xevm_set_vbus(struct musb *musb, int is_on)
{
	struct musb_hdrc_platform_data *usb_data =	\
		musb->controller->platform_data;
		struct tnetv107x_musb_data  *tnetv_bdata = usb_data->board_data;
	int flags;

	WARN_ON(is_on && is_peripheral_active(musb));

	if (is_on)
		is_on = 1;

	spin_lock_irqsave(&tnetv_bdata->lock, flags);
	tnetv_bdata->is_on = is_on;
	spin_unlock_irqrestore(&tnetv_bdata->lock, flags);

	INIT_WORK(&tnetv_bdata->vbus_work, tnetvevm_deferred_drvvbus);

	schedule_work(&tnetv_bdata->vbus_work);
}

static inline int phy_ctrl(int controller, int turn_on)
{
	void __iomem	*phyctl;
	void __iomem	*phyreset;

	u32 phyctrl_val;
	u32 phyreset_val;
	int status = -ENOMEM;
	unsigned long flags;

	/*sanity check arguments */
	if (turn_on != (0 || 1))
	    return -EINVAL;

	if (controller != (0 || 1))
	    return -EINVAL;

	phyreset = ioremap(USB_PHY_RESET, 4);
	if (!phyreset) {
		pr_err("Unable to map USB PHY reset register\n");
		goto error0;
	}

	phyctl = ioremap(USB_PHY_CTRL, 4);
	if (!phyctl) {
		pr_err("Unable to map USB PHY CTL register\n");
		goto error1;
	}

	spin_lock_irqsave(&tnetv107x_phy_lock, flags);

	phyreset_val = __raw_readl(phyreset);
	phyctrl_val = __raw_readl(phyctl);

	/* zero reserved bits */
	phyctrl_val &= 0x7F7F;

	if (turn_on) {
		/* take phy out of reset */
		phyreset_val &= ~(0x1 << controller);
		__raw_writel(phyreset_val, phyreset);

		/*
		 * Start the on-chip PHY and its PLL.
		 */

		phyctrl_val |= PHYPLLON(controller) | SENSEEN(controller)
			| VBUSDETEN(controller);

		phyctrl_val &= ~(PHYPWRDN(controller) | OTGPWRDN(controller));

		__raw_writel(phyctrl_val, phyctl);

		pr_info("Waiting for USB%d PHY clock good...\n",controller);

		while (!(__raw_readl(phyctl) & PHYCLKGD(controller)))
			cpu_relax();
	} else {
		/* Power down the on-chip PHY. */
		phyctrl_val |= PHYPWRDN(controller) | OTGPWRDN(controller);
		__raw_writel(phyctrl_val, phyctl);

		/* put phy in reset */
		phyreset_val |= (0x1 << controller);
		__raw_writel(phyreset_val, phyreset);
	}

	spin_unlock_irqrestore(&tnetv107x_phy_lock, flags);
	status = 0;
	iounmap(phyctl);
error1:
	iounmap(phyreset);
error0:
	return status;
}

/**
 * musb_platform_enable - enable interrupts
 */
void musb_platform_enable(struct musb *musb)
{
	void __iomem *reg_base = musb->ctrl_base;
	u32 epmask;

	/* Workaround: setup IRQs through both register sets. */
	epmask = (((musb->epmask & TX_EP_MASK) << USB_INTR_TX_SHIFT) |
		((musb->epmask & RX_EP_MASK) << USB_INTR_RX_SHIFT));

	musb_writel(reg_base, EP_INTR_MASK_SET_REG, epmask);
	musb_writel(reg_base, CORE_INTR_MASK_SET_REG, USB_INTR_USB_MASK);

	/* Force the DRVVBUS IRQ so we can start polling for ID change. */
	if (is_otg_enabled(musb))
		musb_writel(reg_base, CORE_INTR_SRC_SET_REG,
			    USB_INTR_DRVVBUS << USB_INTR_USB_SHIFT);
}

/**
 * musb_platform_disable - disable HDRC and flush interrupts
 */
void musb_platform_disable(struct musb *musb)
{
	void __iomem *reg_base = musb->ctrl_base;

        musb_writel(reg_base, CORE_INTR_MASK_CLEAR_REG, USB_INTR_USB_MASK);
        musb_writel(reg_base, EP_INTR_MASK_CLEAR_REG,
                         TX_INTR_MASK | RX_INTR_MASK);
        musb_writeb(musb->mregs, MUSB_DEVCTL, 0);
        musb_writel(reg_base, USB_END_OF_INTR_REG, 0);
}

#ifdef CONFIG_USB_MUSB_HDRC_HCD
#define portstate(stmt) 	stmt
#else
#define portstate(stmt)
#endif

#define	POLL_SECONDS	2

static void otg_timer(unsigned long _musb)
{
	struct musb		*musb = (void *)_musb;
	struct musb_hdrc_platform_data *pdata = musb->controller->platform_data;
	struct tnetv107x_musb_data *ptnetv_musb_data = pdata->board_data;
	void __iomem		*mregs = musb->mregs;
	u8			devctl;
	unsigned long		flags;

	/*
	 * We poll because DaVinci's won't expose several OTG-critical
	 * status change events (from the transceiver) otherwise.
	 */
	devctl = musb_readb(mregs, MUSB_DEVCTL);
	DBG(7, "Poll devctl %02x (%s)\n", devctl, otg_state_string(musb));
	spin_lock_irqsave(&musb->lock, flags);
	switch (musb->xceiv->state) {
	case OTG_STATE_A_WAIT_BCON:
		devctl &= ~MUSB_DEVCTL_SESSION;
		musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);

		devctl = musb_readb(musb->mregs, MUSB_DEVCTL);
		if (devctl & MUSB_DEVCTL_BDEVICE) {
			musb->xceiv->state = OTG_STATE_B_IDLE;
			MUSB_DEV_MODE(musb);
		} else {
			musb->xceiv->state = OTG_STATE_A_IDLE;
			MUSB_HST_MODE(musb);
		}
		break;
	case OTG_STATE_A_WAIT_VFALL:
		/*
		 * Wait till VBUS falls below SessionEnd (~0.2 V); the 1.3
		 * RTL seems to mis-handle session "start" otherwise (or in
		 * our case "recover"), in routine "VBUS was valid by the time
		 * VBUSERR got reported during enumeration" cases.
		 */
		if (devctl & MUSB_DEVCTL_VBUS) {
			mod_timer(&(ptnetv_musb_data->otg_workaround), jiffies + POLL_SECONDS * HZ);
			break;
		}

		musb->xceiv->state = OTG_STATE_A_WAIT_VRISE;
		musb_writel(musb->ctrl_base, CORE_INTR_SRC_SET_REG,
			    MUSB_INTR_VBUSERROR << USB_INTR_USB_SHIFT);
		break;
	case OTG_STATE_B_IDLE:
		if (!is_peripheral_enabled(musb))
			break;

		/*
		 * There's no ID-changed IRQ, so we have no good way to tell
		 * when to switch to the A-Default state machine (by setting
		 * the DEVCTL.Session bit).
		 *
		 * Workaround:  whenever we're in B_IDLE, try setting the
		 * session flag every few seconds.  If it works, ID was
		 * grounded and we're now in the A-Default state machine.
		 *
		 * NOTE: setting the session flag is _supposed_ to trigger
		 * SRP but clearly it doesn't.
		 */
		musb_writeb(mregs, MUSB_DEVCTL, devctl | MUSB_DEVCTL_SESSION);
		devctl = musb_readb(mregs, MUSB_DEVCTL);
		if (devctl & MUSB_DEVCTL_BDEVICE)
			mod_timer(&ptnetv_musb_data->otg_workaround, jiffies + POLL_SECONDS * HZ);
		else
			musb->xceiv->state = OTG_STATE_A_IDLE;
		break;
	default:
		break;
	}
	spin_unlock_irqrestore(&musb->lock, flags);
}

void musb_platform_try_idle(struct musb *musb, unsigned long timeout)
{
	static unsigned long last_timer;
	struct musb_hdrc_platform_data *pdata = musb->controller->platform_data;
	struct tnetv107x_musb_data *ptnetv_musb_data = pdata->board_data;

	if (!is_otg_enabled(musb))
		return;

	if (timeout == 0)
		timeout = jiffies + msecs_to_jiffies(3);

	/* Never idle if active, or when VBUS timeout is not set as host */
	if (musb->is_active || (musb->a_wait_bcon == 0 &&
				musb->xceiv->state == OTG_STATE_A_WAIT_BCON)) {
		DBG(4, "%s active, deleting timer\n", otg_state_string(musb));
		del_timer(&(ptnetv_musb_data->otg_workaround));
		last_timer = jiffies;
		return;
	}

	if (time_after(last_timer, timeout) && timer_pending(&ptnetv_musb_data->otg_workaround)) {
		DBG(4, "Longer idle timer already pending, ignoring...\n");
		return;
	}
	last_timer = timeout;

	DBG(4, "%s inactive, starting idle timer for %u ms\n",
	    otg_state_string(musb), jiffies_to_msecs(timeout - jiffies));
	mod_timer(&ptnetv_musb_data->otg_workaround, timeout);
}

static irqreturn_t tnetv107x_interrupt(int irq, void *hci)
{
	struct musb  *musb = hci;
	struct musb_hdrc_platform_data *pdata = musb->controller->platform_data;
	struct tnetv107x_musb_data *ptnetv_musb_data = pdata->board_data;
	void __iomem *reg_base = musb->ctrl_base;
	unsigned long flags;
	irqreturn_t ret = IRQ_NONE;
	u32 epintr, usbintr;

	spin_lock_irqsave(&musb->lock, flags);

	/*
	 * NOTE: TNETV107x shadows the Mentor IRQs.  Don't manage them through
	 * the Mentor registers (except for setup), use the TI ones and EOI.
	 */

	/* Acknowledge and handle non-CPPI interrupts */
	/* Get endpoint interrupts */
	epintr = musb_readl(reg_base, EP_INTR_SRC_MASKED_REG);

	if (epintr) {
		musb_writel(reg_base, EP_INTR_SRC_CLEAR_REG, epintr);

		musb->int_rx =
			(epintr & RX_INTR_MASK) >> USB_INTR_RX_SHIFT;
		musb->int_tx =
			(epintr & TX_INTR_MASK) >> USB_INTR_TX_SHIFT;
	}

	/* Get usb core interrupts */
	usbintr = musb_readl(reg_base, CORE_INTR_SRC_MASKED_REG);
	if (!usbintr && !epintr)
		goto eoi;

	if (usbintr) {
		musb_writel(reg_base, CORE_INTR_SRC_CLEAR_REG, usbintr);

		musb->int_usb =
			(usbintr & USB_INTR_USB_MASK) >> USB_INTR_USB_SHIFT;
	}
	/*
	 * DRVVBUS IRQs are the only proxy we have (a very poor one!) for
	 * TNETV107x missing ID change IRQ.  We need an ID change IRQ to
	 * switch appropriately between halves of the OTG state machine.
	 * Managing DEVCTL.SESSION per Mentor docs requires that we know its
	 * value but DEVCTL.BDEVICE is invalid without DEVCTL.SESSION set.
	 * Also, DRVVBUS pulses for SRP (but not at 5V) ...
	 */
	if (usbintr & (USB_INTR_DRVVBUS << USB_INTR_USB_SHIFT)) {
		int drvvbus = musb_readl(reg_base, USB_STAT_REG);
		void __iomem *mregs = musb->mregs;
		u8 devctl = musb_readb(mregs, MUSB_DEVCTL);
		int err;

		err = is_host_enabled(musb) && (musb->int_usb &
						MUSB_INTR_VBUSERROR);
		if (err) {
			/*
			 * The Mentor core doesn't debounce VBUS as needed
			 * to cope with device connect current spikes. This
			 * means it's not uncommon for bus-powered devices
			 * to get VBUS errors during enumeration.
			 *
			 * This is a workaround, but newer RTL from Mentor
			 * seems to allow a better one: "re"-starting sessions
			 * without waiting for VBUS to stop registering in
			 * devctl.
			 */
			musb->int_usb &= ~MUSB_INTR_VBUSERROR;
			musb->xceiv->state = OTG_STATE_A_WAIT_VFALL;
			mod_timer(&ptnetv_musb_data->otg_workaround, jiffies + POLL_SECONDS * HZ);
			WARNING("VBUS error workaround (delay coming)\n");
		} else if (is_host_enabled(musb) && drvvbus) {
			musb->is_active = 1;
			MUSB_HST_MODE(musb);
			musb->xceiv->default_a = 1;
			musb->xceiv->state = OTG_STATE_A_WAIT_VRISE;
			portstate(musb->port1_status |= USB_PORT_STAT_POWER);
			del_timer(&ptnetv_musb_data->otg_workaround);
		} else {
			musb->is_active = 0;
			MUSB_DEV_MODE(musb);
			musb->xceiv->default_a = 0;
			musb->xceiv->state = OTG_STATE_B_IDLE;
			portstate(musb->port1_status &= ~USB_PORT_STAT_POWER);
		}

		tnetv107xevm_set_vbus(musb, drvvbus);

		DBG(2, "VBUS %s (%s)%s, devctl %02x\n",
				drvvbus ? "on" : "off",
				otg_state_string(musb),
				err ? " ERROR" : "",
				devctl);

		ret = IRQ_HANDLED;
	}

	if (musb->int_tx || musb->int_rx || musb->int_usb)
		ret = musb_interrupt(musb);

 eoi:
	/* EOI needs to be written for the IRQ to be re-asserted. */
	if (ret == IRQ_HANDLED || epintr || usbintr) {
		/* write EOI */
		musb_writel(reg_base, USB_END_OF_INTR_REG, 0);
	}

	/* Poll for ID change */
	if (is_otg_enabled(musb) && musb->xceiv->state == OTG_STATE_B_IDLE)
		mod_timer(&ptnetv_musb_data->otg_workaround, jiffies + POLL_SECONDS * HZ);

	spin_unlock_irqrestore(&musb->lock, flags);

	return ret;

}


int musb_platform_set_mode(struct musb *musb, u8 musb_mode)
{
	return -EIO;
}

int __init musb_platform_init(struct musb *musb, void *board_data)
{
	struct platform_device  *pdev;
	struct musb_hdrc_platform_data *pdata;
	struct tnetv107x_musb_data *ptnetv_musb_data;
	void __iomem *reg_base = musb->ctrl_base;
	u32 rev;
	int ret = -ENODEV, power;
	pdev = to_platform_device(musb->controller);
	pdata = musb->controller->platform_data;

	usb_nop_xceiv_register();
	musb->xceiv = otg_get_transceiver();
	if (!musb->xceiv)
		goto fail;

	musb->mregs += MENTOR_CORE_OFFSET;

	clk_enable(musb->clock);
	pr_debug("reg_base: %p, USB_REVISION_REG: %p, musb->mregs: %x\n", reg_base, USB_REVISION_REG, musb->mregs);
	/* Returns zero if e.g. not clocked */
	rev = musb_readl(reg_base, USB_REVISION_REG);
	if (!rev)
		goto fail1;

	ptnetv_musb_data = kmalloc(sizeof(struct tnetv107x_musb_data), GFP_KERNEL);
	if (!ptnetv_musb_data) {
		ret = -ENOMEM;
		goto fail1;
	}

	spin_lock_init(&ptnetv_musb_data->lock);

	if (is_host_enabled(musb))
		setup_timer(&ptnetv_musb_data->otg_workaround, otg_timer, (unsigned long) musb);

	if(pdata->set_vbus != NULL) {
		musb->board_set_vbus = pdata->set_vbus;
	} else {
		/*default regulator based approach */
		pdata->board_data = ptnetv_musb_data;
		/* get the regulator */
		ptnetv_musb_data->vbus_regulator =
			regulator_get(musb->controller, "vbus");

		if (WARN(IS_ERR(ptnetv_musb_data->vbus_regulator)
			 , "Unable to obtain voltage regulator for USB;"))
			goto fail2;

		if(regulator_is_enabled(ptnetv_musb_data->vbus_regulator)) {
			/* the reg framework can leave regulators on during
			   bootup, leading to unbalanced enable/disables */
			regulator_enable(ptnetv_musb_data->vbus_regulator);
			ptnetv_musb_data->vbus_state = 1;
		} else {
			ptnetv_musb_data->vbus_state = 0;
		}

		/* mA/2 -> uA */
		power = (pdata->power * 2) * 1000;
		if (IS_ERR(regulator_set_current_limit(ptnetv_musb_data->vbus_regulator,
						       power, power)))
			WARNING("Unable to set current limit %duA\n",power);

		musb->board_set_vbus = tnetv107xevm_set_vbus;
	}

	/* Reset the controller */
	musb_writel(reg_base, USB_CTRL_REG, USB_SOFT_RESET_MASK);

	/* Start the on-chip PHY and its PLL. */
	if (phy_ctrl(pdev->id, 1) < 0)
		goto fail2;

	musb->board_set_vbus(musb, 1);

	msleep(5);

	/* NOTE: IRQs are in mixed mode, not bypass to pure MUSB */
	pr_debug("TNETV107x OTG revision %08x, control %02x\n",
		 rev, musb_readb(reg_base, USB_CTRL_REG));

	musb->isr = tnetv107x_interrupt;
	return 0;

fail2:
	kfree(ptnetv_musb_data);
fail1:
	clk_disable(musb->clock);
	usb_nop_xceiv_unregister();
fail:
	return ret;
}

int musb_platform_exit(struct musb *musb)
{
	struct platform_device  *pdev = to_platform_device(musb->controller);
	struct musb_hdrc_platform_data *pdata = musb->controller->platform_data;
	struct tnetv107x_musb_data *ptnetv_musb_data = pdata->board_data;

	if (is_host_enabled(musb))
		del_timer_sync(&ptnetv_musb_data->otg_workaround);

	/* turn off vbus */
	musb->board_set_vbus(musb, 0);

	/* Delay to avoid problems with module reload... */
	if (is_host_enabled(musb) && musb->xceiv->default_a) {
		u8 devctl, warn = 0;
		int delay;

		/*
		 * If there's no peripheral connected, VBUS can take a
		 * long time to fall...
		 */
		for (delay = 30; delay > 0; delay--) {
			devctl = musb_readb(musb->mregs, MUSB_DEVCTL);
			if (!(devctl & MUSB_DEVCTL_VBUS))
				goto done;
			if ((devctl & MUSB_DEVCTL_VBUS) != warn) {
				warn = devctl & MUSB_DEVCTL_VBUS;
				DBG(1, "VBUS %d\n",
					warn >> MUSB_DEVCTL_VBUS_SHIFT);
			}
			msleep(1000);
		}

		/* In OTG mode, another host might be connected... */
		DBG(1, "VBUS off timeout (devctl %02x)\n", devctl);
	}

	/* power down the phy */
	phy_ctrl(pdev->id, 0);

	if(pdata->set_vbus == NULL) {
		flush_scheduled_work();
		/* free the regulator */
		regulator_put(ptnetv_musb_data->vbus_regulator);
	}

done:
	kfree(ptnetv_musb_data);
	clk_disable(musb->clock);
	usb_nop_xceiv_unregister();
	return 0;
}





#ifdef CONFIG_USB_TI_CPPI41_DMA

/*
 * CPPI 4.1 resources used for USB OTG controller module:
 *
 * USB   DMA  DMA  QMgr  Tx     Src
 *       Tx   Rx         QNum   Port
 * ---------------------------------
 * EP0   0    0    0     16,17  1
 * ---------------------------------
 * EP1   1    1    0     18,19  2
 * ---------------------------------
 * EP2   2    2    0     20,21  3
 * ---------------------------------
 * EP3   3    3    0     22,23  4
 * ---------------------------------
 */

static const u16 tx_comp_q[] = { 24, 25 };
static const u16 rx_comp_q[] = { 26, 27 };

const struct usb_cppi41_info usb_cppi41_info = {
        .dma_block      = 0,
        .ep_dma_ch      = { 0, 1, 2, 3 },
        .q_mgr          = 0,
        .num_tx_comp_q  = 2,
        .num_rx_comp_q  = 2,
        .tx_comp_q      = tx_comp_q,
        .rx_comp_q      = rx_comp_q
};

#endif /* CONFIG_USB_TI_CPPI41_DMA */

