/*
 * TNETV107X internal LCD controller
 *
 * Copyright (C) 2010, Texas Instruments
 * Author: Cyril Chemparathy <cyril@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

/*
 * This driver logically breaks mutex_lock into three distinct layers as follows:
 *
 * 1. Frame Buffer Layer - Implements the top level interfaces required
 *    to plug into the Linux framebuffer infrastructure.  This layer
 *    is maintained reasonably agnostic of the controller programming
 *    details
 *
 * 2. LCD Controller Layer - Implements a relatively straight forward state
 *    machine that programs the LCD controller, sets up DMAs, etc.  This layer
 *    glues into the Frame Buffer Layer through a set of well defined
 *    interfaces (below)
 *
 * 3. LCD Panel Layer - All panel specific data (timing, modes, etc.) and
 *    control mechanisms (enable, disable, ...) are abstracted out into this
 *    layer.  Technically, adding support for another panel should be as
 *    simple as implementing an instance of the panel interface.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/fb.h>
#include <linux/mutex.h>
#include <linux/dma-mapping.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/vmalloc.h>
#include <linux/clk.h>
#include <linux/io.h>

#include <mach/tnetv107x.h>

/* Bits and Masks for External LCD Panel Configuration */
#define LCDC_INV_VSYNC		BIT(0)
#define LCDC_INV_HSYNC		BIT(1)
#define LCDC_INV_PIX_CLOCK	BIT(2)
#define LCDC_INV_OUTPUT_EN	BIT(3)
#define LCDC_HSVS_FALLING_EDGE	BIT(4)
#define LCDC_HSVS_CONTROL	BIT(5)
#define LCDC_SIGNAL_MASK	0x003f
#define LCDC_PANEL_TFT		BIT(8)

#define res_size(_r) (((_r)->end - (_r)->start) + 1)

/* LCD Controller Registers */
#define REG_REVISION		0x00
#define REG_CONTROL		0x04
#define REG_STATUS		0x08
#define REG_RASTER_CONTROL	0x28
#define REG_TIMING0		0x2c
#define REG_TIMING1		0x30
#define REG_TIMING2		0x34
#define REG_SUBPANEL1		0x38
#define REG_SUBPANEL2		0x3c
#define REG_DMA_CONTROL		0x40
#define REG_FB0_BASE		0x44
#define REG_FB0_CEILING		0x48
#define REG_FB1_BASE		0x4c
#define REG_FB1_CEILING		0x50

#define CTRL_DIV_MASK		0xff00
#define CTRL_RASTER_MODE	0x0001
#define DMA_BURST_LEN(x)	((x) << 4)
#define DMA_EOF_INTEN		(1   << 2)
#define DMA_DUAL_BUFFER		(1   << 0)

/* Status Register Bits */
#define LCDC_STAT_SYNC_LOST		BIT(2)
#define LCDC_STAT_AC_BIAS		BIT(3)
#define LCDC_STAT_FUF			BIT(5)
#define LCDC_STAT_PALETTE_LOADED	BIT(6)
#define LCDC_STAT_FRAME_DONE		BIT(0)
#define LCDC_STAT_FRAME_DONE0		BIT(8)
#define LCDC_STAT_FRAME_DONE1		BIT(9)

/* Raster Control Register Bits */
#define LCDC_CTRL_LCD_EN		BIT(0)
#define LCDC_CTRL_LCD_TFT		BIT(7)
#define LCDC_CTRL_LCD_TFT_24		BIT(25)
#define LCDC_CTRL_LCD_STN_565		BIT(24)
#define LCDC_CTRL_LINE_IRQ_CLR_SEL	BIT(10)
#define LCDC_CTRL_RASTER_ORDER	BIT(8)

/* Raster Control Register IRQ Bits */
#define LCDC_IRQ_AC_BIAS		BIT(2)
#define LCDC_IRQ_FRAME_DONE		BIT(3)
#define LCDC_IRQ_PALETTE_LOADED		BIT(4)
#define LCDC_IRQ_SYNC_LOST		BIT(5)
#define LCDC_IRQ_FUF			BIT(6)
#define LCDC_IRQ_MASK			(0x1f << 2)
#define LCDC_LOAD_MODE_MASK		(0x3 << 20)
#define LCDC_LOAD_PALETTE		BIT(20)
#define LCDC_LOAD_FRAME			(BIT(20) | BIT(21))

#define MAX_PALETTE_SIZE		PAGE_SIZE

/* Register Access Macros */
#define __LCDC_READ(lcdc, reg)				\
	__raw_readl((lcdc)->virt_base + REG_##reg)

#define LCDC_READ(lcdc, reg) ({				\
	u32 val = __LCDC_READ(lcdc, reg);		\
	dev_dbg(lcdc->dev, "RD(" #reg ") %x\n", val);	\
	val;						\
})

#define __LCDC_WRITE(lcdc, reg, val)			\
	__raw_writel(val, (lcdc)->virt_base + REG_##reg)

#define LCDC_WRITE(lcdc, reg, val) do {			\
	__LCDC_WRITE(lcdc, reg, val);			\
	dev_dbg(lcdc->dev, "WR(" #reg ") %x\n", val);	\
} while (0)

#define LCDC_RMW(lcdc, reg, mask, bits) do {		\
	u32 oldval = __LCDC_READ(lcdc, reg);		\
	u32 newval = (oldval & ~(mask))|(bits);		\
	__LCDC_WRITE(lcdc, reg, newval);		\
	dev_dbg(lcdc->dev, "LCDC_RMW(" #reg ") %x -> %x\n",\
			oldval, newval);		\
} while(0)

#define LCDC_CLEAR(lcdc, reg, bits)	LCDC_RMW(lcdc, reg, bits, 0)
#define LCDC_SET(lcdc, reg, bits)	LCDC_RMW(lcdc, reg, 0, bits)

/* Color modes understood by the controller */
enum tnetv107x_fb_color_format {
	FB_COLOR_CLUT_1BPP,
	FB_COLOR_CLUT_2BPP,
	FB_COLOR_CLUT_4BPP,
	FB_COLOR_CLUT_8BPP,
	FB_COLOR_RGB444,
	FB_COLOR_RGB565,
	FB_COLOR_RGB888,
};

/* FB Driver states */
enum tnetv107x_fb_state {
	FB_DISABLED	= 0,
	/* Anything in between is some initialization state */
	FB_SUSPENDED	= 99,
	FB_ACTIVE	= 100
};

/* States for the LCD Controller State Machine */
enum tnetv107x_state {
	LCDC_STATE_SUSPENDED = 0,
	LCDC_STATE_LOADING_PALETTE,
	LCDC_STATE_ACTIVE,
	LCDC_STATE_SYNC_LOST,
	LCDC_STATE_WAIT_FRAME_PALETTE,
};

/* Input Events for the LCD Controller State Machine */
enum tnetv107x_lcdc_event {
	LCDC_EVENT_ENTER,
	LCDC_EVENT_EXIT,
	LCDC_EVENT_SUSPEND,
	LCDC_EVENT_RESUME,
	LCDC_EVENT_UPDATE_PALETTE,
	LCDC_EVENT_SYNC_LOSS_RECOVER,
	LCDC_EVENT_IRQ_FRAME_DONE,
	LCDC_EVENT_IRQ_FRAME_DONE0,
	LCDC_EVENT_IRQ_FRAME_DONE1,
	LCDC_EVENT_IRQ_PALETTE_LOADED,
	LCDC_EVENT_IRQ_SYNC_LOST,
};

/* Events that get reported during debug */
#define LCDC_VERBOSE_EVENTS  	  	  	  \
	((1 << LCDC_EVENT_SUSPEND)		| \
	 (1 << LCDC_EVENT_RESUME)		| \
	 (1 << LCDC_EVENT_UPDATE_PALETTE)	| \
	 (1 << LCDC_EVENT_SYNC_LOSS_RECOVER)	| \
	 (1 << LCDC_EVENT_IRQ_PALETTE_LOADED)	| \
	 (1 << LCDC_EVENT_IRQ_SYNC_LOST))

/* IRQ Events that can be controlled by (enable/disable)_irq_events */
#define LCDC_IRQ_EVENTS		  	  \
	((1 << LCDC_EVENT_IRQ_FRAME_DONE)	| \
	 (1 << LCDC_EVENT_IRQ_FRAME_DONE0)	| \
	 (1 << LCDC_EVENT_IRQ_FRAME_DONE1)	| \
	 (1 << LCDC_EVENT_IRQ_PALETTE_LOADED)	| \
	 (1 << LCDC_EVENT_IRQ_SYNC_LOST))


struct tnetv107x_lcd_panel;
struct tnetv107x_lcdc;

/* Framebuffer Data */
struct tnetv107x_fb_device {
	int				state;
	struct mutex			rqueue_mutex;
	int				palette_size;
	u32				pseudo_palette[17];
	struct tnetv107x_lcd_panel	*panel;
	struct device			*dev;
	struct platform_device		*pdev;
	dma_addr_t			vram_phys;
	void				*vram_virt;
	unsigned long			vram_size;
	int				vram_alloc:1;
	struct resource			*vram_res;
	enum tnetv107x_fb_color_format	color_mode;
	struct fb_info			*fb_info;
	struct tnetv107x_lcdc		*lcdc;
	struct tnetv107x_fb_data	data;
	struct fb_var_screeninfo	cur_var;
};

/* LCD Controller Data */
struct tnetv107x_lcdc {
	void __iomem			*virt_base;
	u32				phys_base;
	struct resource			*mem_res;
	struct device			*dev;
	struct clk			*clk;
	int				lcdc_irq;
	struct tnetv107x_fb_device	*fbdev;
	void				*palette_virt;
	dma_addr_t			palette_phys;
	int				palette_code;
	int				palette_size;
	enum tnetv107x_state	state;
	wait_queue_head_t		state_wait;
	u32				irq_event_mask;
	struct delayed_work		sync_lost_work;
#define work_to_lcdc(work) container_of(to_delayed_work(work), \
					struct tnetv107x_lcdc, \
					sync_lost_work)
	spinlock_t			lock_hw;
	struct mutex			lock_api;
	u32				dma_start, dma_end;
	struct atomic_notifier_head	sync_notifier;
};

/* LCD Panel Info and Control Interface */
struct tnetv107x_lcd_panel {
	const char	*name;
	int		config;		/* TFT/STN, signal inversion */
	int		bpp;		/* Pixel format in fb mem */
	int		data_lines;	/* Lines on LCD HW interface */

	int		width, height;	/* In millimeters */
	int		x_res, y_res;
	int		pixel_clock;	/* In pico-secs */
	int		hsw;		/* Horizontal synchronization
					   pulse width */
	int		hfp;		/* Horizontal front porch */
	int		hbp;		/* Horizontal back porch */
	int		vsw;		/* Vertical synchronization
					   pulse width */
	int		vfp;		/* Vertical front porch */
	int		vbp;		/* Vertical back porch */
	int		acb;		/* ac-bias pin frequency */
	int		pcd;		/* pixel clock divider.
					   Obsolete use pixel_clock instead */

	int		(*init)		(struct tnetv107x_lcd_panel *panel,
					 struct tnetv107x_fb_device *fbdev);
	int		(*cleanup)	(struct tnetv107x_lcd_panel *panel,
					 struct tnetv107x_fb_device *fbdev);
	int		(*enable)	(struct tnetv107x_lcd_panel *panel,
					 struct tnetv107x_fb_device *fbdev);
	int		(*disable)	(struct tnetv107x_lcd_panel *panel,
					 struct tnetv107x_fb_device *fbdev);

	void		*priv;		/* Panel private data */
};

/* State machine event handlers */
typedef int (*tnetv107x_state_handler)(
		struct tnetv107x_lcdc* lcdc, enum tnetv107x_lcdc_event event);

static int tnetv107x_lcdc_setup(struct tnetv107x_lcdc* lcdc);
static int tnetv107x_lcdc_load_palette(struct tnetv107x_lcdc* lcdc);
static int tnetv107x_lcdc_load_frame(struct tnetv107x_lcdc* lcdc);
static int tnetv107x_lcdc_sync_lost(struct tnetv107x_lcdc* lcdc);

static int tnetv107x_state_suspended(
		struct tnetv107x_lcdc* lcdc, enum tnetv107x_lcdc_event event);
static int tnetv107x_state_loading_palette(
		struct tnetv107x_lcdc* lcdc, enum tnetv107x_lcdc_event event);
static int tnetv107x_state_active(
		struct tnetv107x_lcdc* lcdc, enum tnetv107x_lcdc_event event);
static int tnetv107x_state_sync_lost(
		struct tnetv107x_lcdc* lcdc, enum tnetv107x_lcdc_event event);
static int tnetv107x_state_wait_frame_palette(
		struct tnetv107x_lcdc* lcdc, enum tnetv107x_lcdc_event event);


/****************************************************************************/
/*  LCD Controller State machine framework                                  */
/****************************************************************************/
static const char* tnetv107x_state_names[] = {
	[LCDC_STATE_SUSPENDED]		= "suspended",
	[LCDC_STATE_LOADING_PALETTE]	= "loading_palette",
	[LCDC_STATE_ACTIVE]		= "active",
	[LCDC_STATE_SYNC_LOST]		= "sync_lost",
	[LCDC_STATE_WAIT_FRAME_PALETTE]	= "wait_frame_palette",
};

static tnetv107x_state_handler tnetv107x_state_handlers[] = {
	[LCDC_STATE_SUSPENDED]		= tnetv107x_state_suspended,
	[LCDC_STATE_LOADING_PALETTE]	= tnetv107x_state_loading_palette,
	[LCDC_STATE_ACTIVE]		= tnetv107x_state_active,
	[LCDC_STATE_SYNC_LOST]		= tnetv107x_state_sync_lost,
	[LCDC_STATE_WAIT_FRAME_PALETTE]	= tnetv107x_state_wait_frame_palette,
};



static const char* tnetv107x_lcdc_event_names[] = {
	[LCDC_EVENT_ENTER]		= "enter",
	[LCDC_EVENT_EXIT]		= "exit",
	[LCDC_EVENT_SUSPEND]		= "suspend",
	[LCDC_EVENT_RESUME]		= "resume",
	[LCDC_EVENT_UPDATE_PALETTE]	= "update_palette",
	[LCDC_EVENT_SYNC_LOSS_RECOVER]	= "sync_loss_recover",
	[LCDC_EVENT_IRQ_FRAME_DONE]	= "irq_frame_done",
	[LCDC_EVENT_IRQ_FRAME_DONE0]	= "irq_frame_done_0",
	[LCDC_EVENT_IRQ_FRAME_DONE1]	= "irq_frame_done_1",
	[LCDC_EVENT_IRQ_PALETTE_LOADED]	= "irq_palette_loaded",
	[LCDC_EVENT_IRQ_SYNC_LOST]	= "irq_sync_lost",
};


/* Feed an event into the controller state machine */
static inline int tnetv107x_lcdc_event(struct tnetv107x_lcdc* lcdc,
		enum tnetv107x_lcdc_event event)
{
	int			ret;
	enum tnetv107x_state	state = lcdc->state;

	if ((1 << event) & LCDC_VERBOSE_EVENTS)
		dev_dbg(lcdc->dev, "handling %s event in %s state\n",
				tnetv107x_lcdc_event_names[event],
				tnetv107x_state_names[state]);

	ret = (*tnetv107x_state_handlers[state])(lcdc, event);

	if (ret < 0)
		dev_err(lcdc->dev, "error handling %s event in %s state\n",
				tnetv107x_lcdc_event_names[event],
				tnetv107x_state_names[state]);
	return ret;
}

/* Filter and feed an IRQ event into the controller state machine */
static inline void tnetv107x_lcdc_irq_event(struct tnetv107x_lcdc* lcdc,
		enum tnetv107x_lcdc_event event)
{
	if ((1 << event) & lcdc->irq_event_mask)
		tnetv107x_lcdc_event(lcdc, event);
}

/* Shift to a different state, emit enter/exit events along the way */
static inline int tnetv107x_lcdc_set_state(struct tnetv107x_lcdc* lcdc,
		enum tnetv107x_state newstate)
{
	int			ret;
	enum tnetv107x_state	oldstate = lcdc->state;

	dev_dbg(lcdc->dev, "switching from %s state to %s state\n",
			tnetv107x_state_names[oldstate],
			tnetv107x_state_names[newstate]);

	ret = tnetv107x_lcdc_event(lcdc, LCDC_EVENT_EXIT);
	BUG_ON(lcdc->state != oldstate);
	if (ret < 0) return ret;

	lcdc->state = newstate;

	ret = tnetv107x_lcdc_event(lcdc, LCDC_EVENT_ENTER);
	if (ret < 0) return ret;

	wake_up_all(&lcdc->state_wait);

	return ret;
}

static inline void
tnetv107x_lcdc_enable_irqs(struct tnetv107x_lcdc* lcdc, u32 mask)
{
	u32 oldmask, newmask;

	oldmask = lcdc->irq_event_mask;
	newmask = oldmask | mask;
	lcdc->irq_event_mask = newmask;
	if (newmask && !oldmask)
		enable_irq(lcdc->lcdc_irq);
}

static inline void
tnetv107x_lcdc_enable_irq(struct tnetv107x_lcdc* lcdc,
		enum tnetv107x_lcdc_event event)
{
	tnetv107x_lcdc_enable_irqs(lcdc, 1 << event);
}

static inline void
tnetv107x_lcdc_disable_irqs(struct tnetv107x_lcdc* lcdc, u32 mask)
{
	u32 oldmask, newmask;

	oldmask = lcdc->irq_event_mask;
	newmask = oldmask & ~mask;
	lcdc->irq_event_mask = newmask;
	if (oldmask && !newmask)
		disable_irq_nosync(lcdc->lcdc_irq);
}

static inline void
tnetv107x_lcdc_disable_irq(struct tnetv107x_lcdc* lcdc,
		enum tnetv107x_lcdc_event event)
{
	tnetv107x_lcdc_disable_irqs(lcdc, 1 << event);
}



/*****************************************************************************/
/*  State machine handler routines                                           */
/*****************************************************************************/

static int tnetv107x_state_suspended(struct tnetv107x_lcdc* lcdc,
		enum tnetv107x_lcdc_event event)
{
	int ret = 0;
	struct tnetv107x_lcd_panel* panel = lcdc->fbdev->panel;

	switch (event) {

		case LCDC_EVENT_ENTER:
			if (panel->disable)
				panel->disable(panel, lcdc->fbdev);
			tnetv107x_lcdc_disable_irqs(lcdc, LCDC_IRQ_EVENTS);
			clk_disable(lcdc->clk);
			break;

		case LCDC_EVENT_EXIT:
			break;

		case LCDC_EVENT_SUSPEND:
			break;

		case LCDC_EVENT_RESUME:
			clk_enable(lcdc->clk);
			ret = tnetv107x_lcdc_setup(lcdc);
			if (ret < 0) break;
			if (lcdc->palette_size)
				ret = tnetv107x_lcdc_set_state(lcdc,
						LCDC_STATE_LOADING_PALETTE);
			else
				ret = tnetv107x_lcdc_set_state(lcdc,
						LCDC_STATE_ACTIVE);
			break;

		default:
			ret = -EINVAL;
			break;
	}
	return ret;
}

static int tnetv107x_state_loading_palette(struct tnetv107x_lcdc* lcdc,
		enum tnetv107x_lcdc_event event)
{
	int ret = 0;

	switch (event) {

		case LCDC_EVENT_ENTER:
			tnetv107x_lcdc_enable_irq(lcdc,
					LCDC_EVENT_IRQ_PALETTE_LOADED);
			ret = tnetv107x_lcdc_load_palette(lcdc);
			break;

		case LCDC_EVENT_EXIT:
			tnetv107x_lcdc_disable_irq(lcdc,
					LCDC_EVENT_IRQ_PALETTE_LOADED);
			break;

		case LCDC_EVENT_SUSPEND:
			ret = tnetv107x_lcdc_set_state(lcdc,
					LCDC_STATE_SUSPENDED);
			break;

		case LCDC_EVENT_RESUME:
			break;

		case LCDC_EVENT_IRQ_PALETTE_LOADED:
			LCDC_CLEAR(lcdc, RASTER_CONTROL,
					LCDC_CTRL_LCD_EN);
			ret = tnetv107x_lcdc_set_state(lcdc,
					LCDC_STATE_ACTIVE);
			break;

		case LCDC_EVENT_IRQ_SYNC_LOST:
			ret = tnetv107x_lcdc_sync_lost(lcdc);
			break;

		default:
			ret = -EINVAL;
			break;
	}
	return ret;
}

static int tnetv107x_state_active(struct tnetv107x_lcdc* lcdc,
		enum tnetv107x_lcdc_event event)
{
	int ret = 0;
	struct tnetv107x_lcd_panel* panel = lcdc->fbdev->panel;

	switch (event) {

		case LCDC_EVENT_ENTER:
			tnetv107x_lcdc_enable_irqs(lcdc,
					LCDC_EVENT_IRQ_SYNC_LOST   |
					LCDC_EVENT_IRQ_FRAME_DONE0 |
					LCDC_EVENT_IRQ_FRAME_DONE1);
			ret = tnetv107x_lcdc_load_frame(lcdc);
			if (panel->enable)
				panel->enable(panel, lcdc->fbdev);
			break;

		case LCDC_EVENT_EXIT:
			tnetv107x_lcdc_disable_irqs(lcdc,
					LCDC_EVENT_IRQ_SYNC_LOST   |
					LCDC_EVENT_IRQ_FRAME_DONE0 |
					LCDC_EVENT_IRQ_FRAME_DONE1);
			break;

		case LCDC_EVENT_SUSPEND:
			ret = tnetv107x_lcdc_set_state(lcdc,
					LCDC_STATE_SUSPENDED);
			break;

		case LCDC_EVENT_RESUME:
			break;

		case LCDC_EVENT_IRQ_FRAME_DONE0:
			LCDC_WRITE(lcdc, FB0_BASE,	lcdc->dma_start);
			LCDC_WRITE(lcdc, FB0_CEILING,	lcdc->dma_end);
			atomic_notifier_call_chain(&lcdc->sync_notifier,
							0, NULL);
			break;

		case LCDC_EVENT_IRQ_FRAME_DONE1:
			LCDC_WRITE(lcdc, FB1_BASE,	lcdc->dma_start);
			LCDC_WRITE(lcdc, FB1_CEILING,	lcdc->dma_end);
			atomic_notifier_call_chain(&lcdc->sync_notifier,
							1, NULL);
			break;

		case LCDC_EVENT_UPDATE_PALETTE:
			ret = tnetv107x_lcdc_set_state(lcdc,
					LCDC_STATE_WAIT_FRAME_PALETTE);
			break;

		case LCDC_EVENT_IRQ_SYNC_LOST:
			ret = tnetv107x_lcdc_sync_lost(lcdc);
			break;

		default:
			ret = -EINVAL;
			break;
	}
	return ret;
}

static int tnetv107x_state_sync_lost(struct tnetv107x_lcdc* lcdc,
		enum tnetv107x_lcdc_event event)
{
	int ret = 0;

	switch (event) {

		case LCDC_EVENT_ENTER:
			tnetv107x_lcdc_disable_irqs(lcdc, LCDC_IRQ_EVENTS);
			clk_disable(lcdc->clk);
			break;

		case LCDC_EVENT_EXIT:
			break;

		case LCDC_EVENT_SYNC_LOSS_RECOVER:
			tnetv107x_lcdc_enable_irq(lcdc,
					LCDC_EVENT_IRQ_SYNC_LOST);
			clk_enable(lcdc->clk);
			ret = tnetv107x_lcdc_setup(lcdc);
			if (ret < 0) break;
			if (lcdc->palette_size)
				ret = tnetv107x_lcdc_set_state(lcdc,
						LCDC_STATE_LOADING_PALETTE);
			else
				ret = tnetv107x_lcdc_set_state(lcdc,
						LCDC_STATE_ACTIVE);
			break;

		case LCDC_EVENT_SUSPEND:
			cancel_delayed_work(&lcdc->sync_lost_work);
			ret = tnetv107x_lcdc_set_state(lcdc,
					LCDC_STATE_SUSPENDED);
			break;

		case LCDC_EVENT_RESUME:
			break;

		default:
			ret = -EINVAL;
			break;
	}
	return ret;
}

static int tnetv107x_state_wait_frame_palette(struct tnetv107x_lcdc* lcdc,
		enum tnetv107x_lcdc_event event)
{
	int ret = 0;

	switch (event) {

		case LCDC_EVENT_ENTER:
			tnetv107x_lcdc_enable_irqs(lcdc,
					LCDC_EVENT_IRQ_SYNC_LOST |
					LCDC_EVENT_IRQ_FRAME_DONE);
			break;

		case LCDC_EVENT_EXIT:
			tnetv107x_lcdc_disable_irqs(lcdc,
					LCDC_EVENT_IRQ_FRAME_DONE |
					LCDC_EVENT_IRQ_SYNC_LOST);
			break;

		case LCDC_EVENT_SUSPEND:
			ret = tnetv107x_lcdc_set_state(lcdc,
					LCDC_STATE_SUSPENDED);
			break;

		case LCDC_EVENT_RESUME:
			break;

		case LCDC_EVENT_IRQ_FRAME_DONE:
			LCDC_CLEAR(lcdc, RASTER_CONTROL, LCDC_CTRL_LCD_EN);
			LCDC_WRITE(lcdc, DMA_CONTROL, 0);
			ret = tnetv107x_lcdc_set_state(lcdc,
					LCDC_STATE_LOADING_PALETTE);
			break;

		case LCDC_EVENT_IRQ_SYNC_LOST:
			ret = tnetv107x_lcdc_sync_lost(lcdc);
			break;

		default:
			ret = -EINVAL;
			break;
	}
	return ret;
}



/*****************************************************************************/
/*  Controller programming helper routines                                   */
/*****************************************************************************/

static int tnetv107x_lcdc_setup(struct tnetv107x_lcdc* lcdc) {
	u32 l;
	struct tnetv107x_lcd_panel *panel = lcdc->fbdev->panel;
	struct fb_var_screeninfo *var = &lcdc->fbdev->fb_info->var;
	int is_tft, is_tft_24, is_stn_565;
	unsigned long lck, pck;
	int pck_div;

	switch (lcdc->fbdev->color_mode) {
		case FB_COLOR_CLUT_1BPP:
			lcdc->palette_code = 0x0000;
			lcdc->palette_size = 32;
			break;
		case FB_COLOR_CLUT_2BPP:
			lcdc->palette_code = 0x1000;
			lcdc->palette_size = 32;
			break;
		case FB_COLOR_CLUT_4BPP:
			lcdc->palette_code = 0x2000;
			lcdc->palette_size = 32;
			break;
		case FB_COLOR_CLUT_8BPP:
			lcdc->palette_code = 0x3000;
			lcdc->palette_size = 512;
			break;
		case FB_COLOR_RGB565:
			lcdc->palette_code = 0x4000;
			lcdc->palette_size = 32;
			break;
		case FB_COLOR_RGB444:
			lcdc->palette_code = 0x4000;
			lcdc->palette_size = 32;
			break;
		case FB_COLOR_RGB888:
			lcdc->palette_code = 0x4000;
			lcdc->palette_size = 0;
			break;
		default:
			dev_err(lcdc->dev, "invalid color mode %d\n",
					lcdc->fbdev->color_mode);
			return -EINVAL;
	}

	is_tft = panel->config & LCDC_PANEL_TFT;
	is_tft_24 = is_tft && (lcdc->fbdev->color_mode ==
			FB_COLOR_RGB888);
	is_stn_565 = (!is_tft) && (lcdc->fbdev->color_mode ==
			FB_COLOR_RGB565);

	lck = clk_get_rate(lcdc->clk);
	pck = PICOS2KHZ(var->pixclock) * 1000;
	pck = max(1ul, pck);
	pck_div = (lck / pck) - 1;

	if (is_tft) {
		if (pck_div < 2) {
			dev_warn(lcdc->dev, "pixclock %lu kHz too high.\n",
					pck / 1000);
			pck_div = 2;
		}
	} else {
		if (pck_div < 3) {
			dev_warn(lcdc->dev, "pixclock %lu kHz too high.\n",
					pck / 1000);
			pck_div = 3;
		}
	}

	if (pck_div > 255) {
		dev_warn(lcdc->dev, "pixclock %lu kHz too low.\n",
			 pck / 1000);
		pck_div = 255;
	}

	pck = lck / (pck_div + 1);

	var->pixclock = KHZ2PICOS(pck / 1000); /* Update with real value */

	dev_dbg(lcdc->dev, "module clock @%lukHz, div=%d, pixclock @%lukHz\n",
			lck / 1000, pck_div, pck / 1000);

	LCDC_RMW(lcdc, CONTROL, CTRL_DIV_MASK,
			pck_div << 8 | CTRL_RASTER_MODE);
	udelay(10);	/* Wait for clocks to settle */

	LCDC_RMW(lcdc, RASTER_CONTROL,
			LCDC_CTRL_LCD_TFT		|
			LCDC_CTRL_LCD_TFT_24	|
			LCDC_CTRL_LCD_STN_565,
			(is_tft     ? LCDC_CTRL_LCD_TFT     : 0) |
			(is_tft_24  ? LCDC_CTRL_LCD_TFT_24  : 0) |
			(is_stn_565 ? LCDC_CTRL_LCD_STN_565 : 0) |
			LCDC_CTRL_RASTER_ORDER);

	LCDC_RMW(lcdc, TIMING2, LCDC_SIGNAL_MASK << 20,
			(panel->config & LCDC_SIGNAL_MASK) << 20);

	l = var->xres/16 - 1;
	LCDC_WRITE(lcdc, TIMING0,
			((l & 0x7f) << 4)		|
			((l & 0x80) >> 4)		|
			((var->hsync_len - 1) << 10)	|
			((var->left_margin - 1) << 16)	|
			((var->right_margin - 1) << 24));

	l = var->yres - 1;
	LCDC_WRITE(lcdc, TIMING1,
			(l & 0x3ff)			|
			((var->vsync_len - 1) << 10)	|
			((var->upper_margin) << 16)	|
			((var->lower_margin) << 24));

	return 0;
}

static int tnetv107x_lcdc_load_palette(struct tnetv107x_lcdc* lcdc) {
	u16		*palette;
	u32		start, end;

	if (!lcdc->palette_size) return -EINVAL;

	palette = (u16 *)lcdc->palette_virt;

	*(u16 *)palette &= 0x0fff;
	*(u16 *)palette |= lcdc->palette_code;

	start	= lcdc->palette_phys;
	end	= start + lcdc->palette_size;

	LCDC_WRITE(lcdc, FB0_BASE,	start);
	LCDC_WRITE(lcdc, FB0_CEILING,	end - 1);

	dev_dbg(lcdc->dev, "dma setup for palette load, range %x - %x\n",
			start, end - 1);

	LCDC_WRITE(lcdc, DMA_CONTROL, DMA_BURST_LEN(4));

	LCDC_RMW(lcdc, RASTER_CONTROL,
			LCDC_IRQ_MASK	| LCDC_LOAD_MODE_MASK,
			LCDC_CTRL_LCD_EN   | LCDC_LOAD_PALETTE  |
			LCDC_IRQ_FUF	| LCDC_IRQ_SYNC_LOST |
			LCDC_IRQ_PALETTE_LOADED);

	return 0;
}

static void tnetv107x_lcdc_calc_dma(struct tnetv107x_lcdc* lcdc) {
	struct fb_var_screeninfo	*var = &lcdc->fbdev->fb_info->var;
	struct fb_fix_screeninfo	*fix = &lcdc->fbdev->fb_info->fix;
	u32				start, end;

	start	= lcdc->fbdev->vram_phys +
		  var->yoffset * fix->line_length +
		  var->xoffset * var->bits_per_pixel / 8;
	end	= start + var->yres * fix->line_length - 1;

	lcdc->dma_start	= start;
	lcdc->dma_end	= end;
}

static int tnetv107x_lcdc_load_frame(struct tnetv107x_lcdc* lcdc) {
	tnetv107x_lcdc_calc_dma(lcdc);

	LCDC_WRITE(lcdc, FB0_BASE,	lcdc->dma_start);
	LCDC_WRITE(lcdc, FB0_CEILING,	lcdc->dma_end);
	LCDC_WRITE(lcdc, FB1_BASE,	lcdc->dma_start);
	LCDC_WRITE(lcdc, FB1_CEILING,	lcdc->dma_end);

	dev_dbg(lcdc->dev, "dma setup for frame load, range %x - %x\n",
			lcdc->dma_start, lcdc->dma_end);

	LCDC_WRITE(lcdc, DMA_CONTROL, DMA_BURST_LEN(4)	|
				      DMA_DUAL_BUFFER	|
				      DMA_EOF_INTEN);

	LCDC_RMW(lcdc, RASTER_CONTROL,
			LCDC_IRQ_MASK	| LCDC_LOAD_MODE_MASK,
			LCDC_CTRL_LCD_EN   | LCDC_LOAD_FRAME    |
			LCDC_IRQ_FUF	| LCDC_IRQ_SYNC_LOST |
			LCDC_IRQ_FRAME_DONE);
	return 0;
}

static void tnetv107x_lcdc_sync_lost_work(struct work_struct *work) {
	struct tnetv107x_lcdc* lcdc = work_to_lcdc(work);
	unsigned long flags;

	spin_lock_irqsave(&lcdc->lock_hw, flags);

	tnetv107x_lcdc_event(lcdc, LCDC_EVENT_SYNC_LOSS_RECOVER);

	spin_unlock_irqrestore(&lcdc->lock_hw, flags);
}

static int tnetv107x_lcdc_sync_lost(struct tnetv107x_lcdc* lcdc) {
	int ret;

	ret = tnetv107x_lcdc_set_state(lcdc, LCDC_STATE_SYNC_LOST);
	if (ret < 0) return ret;
	ret = schedule_delayed_work(&lcdc->sync_lost_work, 1);
	return ret;
}

static irqreturn_t tnetv107x_lcdc_interrupt(int irq, void *dev_id) {
	struct tnetv107x_lcdc* lcdc = (struct tnetv107x_lcdc*)dev_id;
	u32 status;
	unsigned long flags;

	spin_lock_irqsave(&lcdc->lock_hw, flags);

	status = __LCDC_READ(lcdc, STATUS);

	__LCDC_WRITE(lcdc, STATUS, status & (
				LCDC_STAT_FUF	 		|
				LCDC_STAT_SYNC_LOST 		|
				LCDC_STAT_PALETTE_LOADED 	|
				LCDC_STAT_AC_BIAS		|
				LCDC_STAT_FRAME_DONE		|
				LCDC_STAT_FRAME_DONE0		|
				LCDC_STAT_FRAME_DONE1));

	if (status & (LCDC_STAT_FUF | LCDC_STAT_SYNC_LOST))
		tnetv107x_lcdc_irq_event(lcdc, LCDC_EVENT_IRQ_SYNC_LOST);
	if (status & LCDC_STAT_PALETTE_LOADED)
		tnetv107x_lcdc_irq_event(lcdc, LCDC_EVENT_IRQ_PALETTE_LOADED);
	if (status & LCDC_STAT_FRAME_DONE)
		tnetv107x_lcdc_irq_event(lcdc, LCDC_EVENT_IRQ_FRAME_DONE);
	if (status & LCDC_STAT_FRAME_DONE0)
		tnetv107x_lcdc_irq_event(lcdc, LCDC_EVENT_IRQ_FRAME_DONE0);
	if (status & LCDC_STAT_FRAME_DONE1)
		tnetv107x_lcdc_irq_event(lcdc, LCDC_EVENT_IRQ_FRAME_DONE1);

	spin_unlock_irqrestore(&lcdc->lock_hw, flags);

	return IRQ_HANDLED;
}


/*****************************************************************************/
/*  LCD Controller Layer interfaces that are used by the FB Layer            */
/*****************************************************************************/

static int tnetv107x_lcdc_suspend(struct tnetv107x_fb_device* fbdev) {
	int ret;
	struct tnetv107x_lcdc* lcdc = fbdev->lcdc;
	unsigned long flags;

	mutex_lock(&lcdc->lock_api);
	spin_lock_irqsave(&lcdc->lock_hw, flags);
	ret = tnetv107x_lcdc_event(lcdc, LCDC_EVENT_SUSPEND);
	spin_unlock_irqrestore(&lcdc->lock_hw, flags);
	mutex_unlock(&lcdc->lock_api);

	return ret;
}

static int tnetv107x_lcdc_resume(struct tnetv107x_fb_device* fbdev) {
	int ret;
	struct tnetv107x_lcdc* lcdc = fbdev->lcdc;
	unsigned long flags;

	mutex_lock(&lcdc->lock_api);
	spin_lock_irqsave(&lcdc->lock_hw, flags);
	ret = tnetv107x_lcdc_event(lcdc, LCDC_EVENT_RESUME);
	spin_unlock_irqrestore(&lcdc->lock_hw, flags);
	mutex_unlock(&lcdc->lock_api);

	return ret;
}

static int tnetv107x_lcdc_change_mode(struct tnetv107x_fb_device* fbdev) {
	int ret;
	struct tnetv107x_lcdc* lcdc = fbdev->lcdc;
	unsigned long flags;

	mutex_lock(&lcdc->lock_api);
	spin_lock_irqsave(&lcdc->lock_hw, flags);
	ret = tnetv107x_lcdc_event(lcdc, LCDC_EVENT_SUSPEND);
	if (ret < 0) goto unlock_ret;
	ret = tnetv107x_lcdc_event(lcdc, LCDC_EVENT_RESUME);

unlock_ret:
	spin_unlock_irqrestore(&lcdc->lock_hw, flags);
	mutex_unlock(&lcdc->lock_api);

	return ret;
}

static int tnetv107x_lcdc_pan_display(struct tnetv107x_fb_device* fbdev) {
	struct tnetv107x_lcdc* lcdc = fbdev->lcdc;
	unsigned long flags;

	mutex_lock(&lcdc->lock_api);
	spin_lock_irqsave(&lcdc->lock_hw, flags);
	tnetv107x_lcdc_calc_dma(lcdc);
	spin_unlock_irqrestore(&lcdc->lock_hw, flags);
	mutex_unlock(&lcdc->lock_api);

	return 0;
}

static int tnetv107x_lcdc_setcolreg(struct tnetv107x_fb_device* fbdev,
		u_int regno, u16 red, u16 green, u16 blue, u16 transp,
		int update_hw_pal)
{
	struct tnetv107x_lcdc*	lcdc = fbdev->lcdc;
	u16			*palette;
	int			ret = 0;
	unsigned long		flags;

	mutex_lock(&lcdc->lock_api);
	spin_lock_irqsave(&lcdc->lock_hw, flags);

	if (!lcdc->palette_size)
		goto unlock_ret;

	ret = -EINVAL;
	if (regno > 255 || regno == 0)
		goto unlock_ret;

	palette = (u16 *)lcdc->palette_virt;

	palette[regno] &= ~0x0fff;
	palette[regno] |= ((red >> 12) << 8) | ((green >> 12) << 4 ) |
			   (blue >> 12);

	if (update_hw_pal)
		ret = tnetv107x_lcdc_event(lcdc, LCDC_EVENT_UPDATE_PALETTE);
	else
		ret = 0;

unlock_ret:
	spin_unlock_irqrestore(&lcdc->lock_hw, flags);
	mutex_unlock(&lcdc->lock_api);
	return ret;
}

static int tnetv107x_lcdc_sync_notifier_register(
		struct tnetv107x_fb_device* fbdev, struct notifier_block* nb)
{
	struct tnetv107x_lcdc*	lcdc = fbdev->lcdc;
	int			ret = 0;
	unsigned long		flags;

	mutex_lock(&lcdc->lock_api);
	spin_lock_irqsave(&lcdc->lock_hw, flags);

	ret = atomic_notifier_chain_register(&lcdc->sync_notifier, nb);

	spin_unlock_irqrestore(&lcdc->lock_hw, flags);
	mutex_unlock(&lcdc->lock_api);
	return ret;
}

static int tnetv107x_lcdc_sync_notifier_unregister(
		struct tnetv107x_fb_device* fbdev, struct notifier_block* nb)
{
	struct tnetv107x_lcdc*	lcdc = fbdev->lcdc;
	int			ret = 0;
	unsigned long		flags;

	mutex_lock(&lcdc->lock_api);
	spin_lock_irqsave(&lcdc->lock_hw, flags);

	ret = atomic_notifier_chain_unregister(&lcdc->sync_notifier, nb);

	spin_unlock_irqrestore(&lcdc->lock_hw, flags);
	mutex_unlock(&lcdc->lock_api);
	return ret;
}



/*****************************************************************************/
/*  LCD Controller Layer initialization and cleanup routines                 */
/*****************************************************************************/

static int tnetv107x_lcdc_alloc_palette(struct tnetv107x_lcdc* lcdc) {
	lcdc->palette_virt = dma_alloc_writecombine(lcdc->dev,
		MAX_PALETTE_SIZE, &lcdc->palette_phys, GFP_KERNEL);
	if (!lcdc->palette_virt) {
		dev_err(lcdc->dev, "failed to alloc palette memory\n");
		return -ENOMEM;
	}
	memset(lcdc->palette_virt, 0, MAX_PALETTE_SIZE);

	return 0;
}

static void tnetv107x_lcdc_free_palette(struct tnetv107x_lcdc* lcdc) {
	dma_free_writecombine(lcdc->dev, MAX_PALETTE_SIZE,
			lcdc->palette_virt, lcdc->palette_phys);
}

static int tnetv107x_lcdc_cleanup(struct tnetv107x_fb_device *fbdev) {
	struct tnetv107x_lcdc *lcdc = fbdev->lcdc;
	unsigned long flags;

	if (!lcdc) return -EINVAL;

	mutex_lock(&lcdc->lock_api);
	spin_lock_irqsave(&lcdc->lock_hw, flags);

	tnetv107x_lcdc_suspend(fbdev);

	tnetv107x_lcdc_free_palette(lcdc);

	clk_put(lcdc->clk);

	if (lcdc->mem_res) {
		release_resource(lcdc->mem_res);
		kfree(lcdc->mem_res);
	}

	if (lcdc->lcdc_irq > 0)
		free_irq(lcdc->lcdc_irq, lcdc);

	fbdev->lcdc = NULL;

	spin_unlock_irqrestore(&lcdc->lock_hw, flags);
	mutex_unlock(&lcdc->lock_api);

	kfree(lcdc);

	return 0;
}

static int tnetv107x_lcdc_init(struct tnetv107x_fb_device *fbdev)
{
	struct platform_device *pdev = fbdev->pdev;
	struct tnetv107x_lcdc *lcdc;
	int ret = 0;
	struct resource *res;
	struct device* dev = &pdev->dev;

	lcdc = kzalloc(sizeof(struct tnetv107x_lcdc), GFP_KERNEL);
	if (!lcdc) {
		dev_err(dev, "cannot allocate device info\n");
		return -ENOMEM;
	}
	lcdc->dev = dev;
	lcdc->fbdev = fbdev;
	fbdev->lcdc = lcdc;

	init_waitqueue_head(&lcdc->state_wait);
	spin_lock_init(&lcdc->lock_hw);
	mutex_init(&lcdc->lock_api);
	ATOMIC_INIT_NOTIFIER_HEAD(&lcdc->sync_notifier);

	lcdc->lcdc_irq = platform_get_irq_byname(pdev, "lcdc_irq");

	ret = -ENODEV;
	if (lcdc->lcdc_irq < 0) {
		dev_err(dev, "cannot determine device interrupt\n");
		goto error;
	}

	ret = -ENODEV;
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
			"lcdc_regs");
	if (!res) {
		dev_err(dev, "cannot determine register area\n");
		goto error;
	}

	ret = -EINVAL;
	lcdc->mem_res = request_mem_region(res->start, res_size(res),
						pdev->name);
	if (!lcdc->mem_res) {
		dev_err(dev, "cannot claim register memory\n");
		goto error;
	}

	lcdc->phys_base = lcdc->mem_res->start;
	lcdc->virt_base = ioremap(lcdc->phys_base, 0xff);

	ret = -EINVAL;
	lcdc->clk = clk_get(dev, NULL);
	if (!lcdc->clk) {
		dev_err(dev, "cannot claim device clock\n");
		goto error;
	}

	clk_set_rate(lcdc->clk, 100*1000*1000);

	lcdc->state = LCDC_STATE_SUSPENDED;
	INIT_DELAYED_WORK(&lcdc->sync_lost_work,
			tnetv107x_lcdc_sync_lost_work);

	if (request_irq(lcdc->lcdc_irq, tnetv107x_lcdc_interrupt,
				IRQF_DISABLED, "tnetv107x-lcdc", lcdc)) {
		dev_err(dev, "Could not allocate lcdc IRQ!\n");
		ret = -EINVAL;
		goto error;
	}
	disable_irq(lcdc->lcdc_irq);

	if ((ret = tnetv107x_lcdc_alloc_palette(lcdc)) < 0)
		goto error;

	return 0;

error:
	tnetv107x_lcdc_cleanup(fbdev);
	return ret;
}



/*****************************************************************************/
/*  LCD Panel Layer - Seiko RA169Z 24-bit 800x480 Panel                      */
/*****************************************************************************/

static int seiko_ra169z_enable(struct tnetv107x_lcd_panel *panel,
		struct tnetv107x_fb_device *fbdev)
{
	int gpio = fbdev->data.panel_data.seiko_ra169z.gpio;

	if (gpio >= 0)
		gpio_set_value(gpio, 1);
	return 0;
}

static int seiko_ra169z_disable(struct tnetv107x_lcd_panel *panel,
		struct tnetv107x_fb_device *fbdev)
{
	int gpio = fbdev->data.panel_data.seiko_ra169z.gpio;

	if (gpio >= 0)
		gpio_set_value(gpio, 0);
	return 0;
}

static int seiko_ra169z_init(struct tnetv107x_lcd_panel *panel,
		struct tnetv107x_fb_device *fbdev)
{
	int gpio = fbdev->data.panel_data.seiko_ra169z.gpio;
	int ret;

	if ((ret = gpio_request(gpio, "seiko_ra169z")) < 0) {
		dev_err(fbdev->dev, "failed to acquire display gpio\n");
		return ret;
	}

	if ((ret = gpio_direction_output(gpio, 0)) < 0) {
		dev_err(fbdev->dev, "failed to set display gpio direction\n");
		gpio_free(gpio);
		return ret;
	}

	return 0;
}

static int seiko_ra169z_cleanup(struct tnetv107x_lcd_panel *panel,
		struct tnetv107x_fb_device *fbdev)
{
	int gpio = fbdev->data.panel_data.seiko_ra169z.gpio;

	if (gpio >= 0) {
		gpio_set_value(gpio, 0);
		gpio_free(gpio);
	}
	return 0;
}

struct tnetv107x_lcd_panel seiko_ra169z = {
	.name		= "seiko_ra169z",
	.config		= LCDC_PANEL_TFT	|
			  LCDC_INV_VSYNC	|
			  LCDC_INV_HSYNC	|
			  LCDC_INV_PIX_CLOCK	|
			  LCDC_HSVS_CONTROL,
	.bpp		= 24,
	.x_res		= 800,
	.y_res		= 480,
	.pixel_clock	= 60000,
	.hsw		= 10,
	.hfp		= 164,
	.hbp		= 89,
	.vsw		= 10,
	.vfp		= 10,
	.vbp		= 23,
	.acb		= 0,
	.init		= seiko_ra169z_init,
	.cleanup	= seiko_ra169z_cleanup,
	.enable		= seiko_ra169z_enable,
	.disable	= seiko_ra169z_disable,
};


/*****************************************************************************/
/*  LCD Panel Layer - Panel list and finder routines                         */
/*****************************************************************************/
struct tnetv107x_lcd_panel *tnetv107x_lcd_panels[] = {
	&seiko_ra169z,
};

static struct tnetv107x_lcd_panel* tnetv107x_panel_find(const char* name) {
	struct tnetv107x_lcd_panel* panel;
	int i;

	for (i = 0; i < ARRAY_SIZE(tnetv107x_lcd_panels); i++) {
		panel = tnetv107x_lcd_panels[i];
		if (strcmp(panel->name, name) == 0)
			return panel;
	}
	return NULL;
}


/*****************************************************************************/
/*  Framebuffer Layer utility routines                                       */
/*****************************************************************************/

static inline enum tnetv107x_fb_color_format
tnetv107x_fb_bpp_to_color_mode(u32 bpp)
{
	enum tnetv107x_fb_color_format color_mode;

	switch (bpp) {
		case 1:  color_mode = FB_COLOR_CLUT_1BPP; break;
		case 2:  color_mode = FB_COLOR_CLUT_2BPP; break;
		case 4:  color_mode = FB_COLOR_CLUT_4BPP; break;
		case 8:  color_mode = FB_COLOR_CLUT_8BPP; break;
		case 12: color_mode = FB_COLOR_RGB444; break;
		case 16: color_mode = FB_COLOR_RGB565; break;
		case 24: color_mode = FB_COLOR_RGB888; break;
		default: color_mode = -EINVAL; break;
	}

	return color_mode;
}

static inline int tnetv107x_fb_color_mode_to_bpp(
		enum tnetv107x_fb_color_format color_mode)
{
	int bpp;

	switch (color_mode) {
		case FB_COLOR_CLUT_1BPP: bpp = 1; break;
		case FB_COLOR_CLUT_2BPP: bpp = 2; break;
		case FB_COLOR_CLUT_4BPP: bpp = 4; break;
		case FB_COLOR_CLUT_8BPP: bpp = 8; break;
		case FB_COLOR_RGB444:	bpp = 16; break;
		case FB_COLOR_RGB565:	bpp = 16; break;
		case FB_COLOR_RGB888:	bpp = 24; break;
		default:		bpp = -EINVAL; break;
	}

	return bpp;

}

static inline int tnetv107x_fb_color_mode_to_visual(
		enum tnetv107x_fb_color_format color_mode)
{
	switch (color_mode) {
		case FB_COLOR_CLUT_1BPP:
		case FB_COLOR_CLUT_2BPP:
		case FB_COLOR_CLUT_4BPP:
		case FB_COLOR_CLUT_8BPP:
			return FB_VISUAL_PSEUDOCOLOR;
		case FB_COLOR_RGB444:
		case FB_COLOR_RGB565:
		case FB_COLOR_RGB888:
			return FB_VISUAL_TRUECOLOR;
		default:
			return -EINVAL;
	}
}

int tnetv107x_fb_sync_notifier_register(struct fb_info *fbi,
		struct notifier_block* nb)
{
	struct tnetv107x_fb_device *fbdev = fbi->par;

	return tnetv107x_lcdc_sync_notifier_register(fbdev, nb);
}

int tnetv107x_fb_sync_notifier_unregister(struct fb_info *fbi,
		struct notifier_block* nb)
{
	struct tnetv107x_fb_device *fbdev = fbi->par;

	return tnetv107x_lcdc_sync_notifier_unregister(fbdev, nb);
}

/*****************************************************************************/
/*  Framebuffer Linux Interface - fb_ops routines                            */
/*****************************************************************************/

/* Nothing to do on open and release */
static int tnetv107x_fb_open(struct fb_info *info, int user) {
	return 0;
}

static int tnetv107x_fb_release(struct fb_info *info, int user) {
	return 0;
}

/*
 * Store a single color palette entry into a pseudo palette or the hardware
 * palette if one is available. For now we support only 16bpp and thus store
 * the entry only to the pseudo palette.
 */
static int __tnetv107x_fb_setcolreg(struct fb_info *fbi, u_int regno,
		u_int red, u_int green, u_int blue, u_int transp,
		int update_hw_pal) {
	struct tnetv107x_fb_device	*fbdev = fbi->par;
	struct fb_var_screeninfo *var = &fbi->var;
	int ret = 0;
	static int clut_size[] = {
		[FB_COLOR_CLUT_8BPP]	= 16,
		[FB_COLOR_CLUT_4BPP]	= 16,
		[FB_COLOR_CLUT_2BPP]	= 4,
		[FB_COLOR_CLUT_1BPP]	= 2,
		[FB_COLOR_RGB444]	= 16,
		[FB_COLOR_RGB565]	= 16,
		[FB_COLOR_RGB888]	= 16,
	};

	BUG_ON(fbdev->color_mode >= ARRAY_SIZE(clut_size));

	if ((regno < 0) || (regno >= clut_size[fbdev->color_mode]))
		return -EINVAL;

	switch (fbdev->color_mode) {
		case FB_COLOR_CLUT_8BPP:
		case FB_COLOR_CLUT_4BPP:
		case FB_COLOR_CLUT_2BPP:
		case FB_COLOR_CLUT_1BPP:
			ret = tnetv107x_lcdc_setcolreg(fbdev, regno,
					red, green, blue, transp,
					update_hw_pal);
			if (ret != 0) break;
			/* Fallthrough */

		case FB_COLOR_RGB565:
		case FB_COLOR_RGB444:
		case FB_COLOR_RGB888:

#define color_field(col, val)			\
	(((u32)val >> (16 - var->col.length))	\
	 	<< var->col.offset)
			((u32*)fbi->pseudo_palette)[regno] =
				color_field(red, red)     |
				color_field(green, green) |
				color_field(blue, blue);
			break;
		default:
			BUG();
	}
	return ret;
}

static int tnetv107x_fb_setcolreg(u_int regno, u_int red, u_int green,
		u_int blue, u_int transp, struct fb_info *info)
{
	return __tnetv107x_fb_setcolreg(info, regno, red, green, blue,
					transp, 1);
}

static int tnetv107x_fb_setcmap(struct fb_cmap *cmap, struct fb_info *info)
{
	int count, index, ret;
	u16 *red, *green, *blue, *transp;
	u16 trans = 0xffff;

	red     = cmap->red;
	green   = cmap->green;
	blue    = cmap->blue;
	transp  = cmap->transp;
	index   = cmap->start;

	for (count = 0; count < cmap->len; count++) {
		if (transp)
			trans = *transp++;
		ret = __tnetv107x_fb_setcolreg(info, index++, *red++, *green++,
				*blue++, trans, count == cmap->len - 1);
		if (ret != 0)
			return ret;
	}

	return 0;
}

static int tnetv107x_fb_blank(int blank, struct fb_info *fbi)
{
	struct tnetv107x_fb_device *fbdev = fbi->par;
	int ret = 0;

	mutex_lock(&fbdev->rqueue_mutex);
	switch (blank) {
		case VESA_NO_BLANKING:
			if (fbdev->state == FB_SUSPENDED) {
				ret = tnetv107x_lcdc_resume(fbdev);
				fbdev->state = FB_ACTIVE;
			}
			break;
		case VESA_POWERDOWN:
			if (fbdev->state == FB_ACTIVE) {
				ret = tnetv107x_lcdc_suspend(fbdev);
				fbdev->state = FB_SUSPENDED;
			}
			break;
		default:
			ret = -EINVAL;
	}
	mutex_unlock(&fbdev->rqueue_mutex);

	return ret;
}

static bool tnetv107x_fb_set_fix(struct fb_info *fbi)
{
	struct fb_fix_screeninfo *fix = &fbi->fix;
	struct fb_var_screeninfo *var = &fbi->var;
	struct tnetv107x_fb_device *fbdev = fbi->par;
	enum tnetv107x_fb_color_format color_mode;
	u32 bpp = var->bits_per_pixel;
	bool changed = false;

	fix->type = FB_TYPE_PACKED_PIXELS;

	color_mode = tnetv107x_fb_bpp_to_color_mode(bpp);
	if (color_mode != fbdev->color_mode)
		changed = true;
	fbdev->color_mode = color_mode;

	BUG_ON(fbdev->color_mode < 0); /* Should be trapped in fb_check_var */
	BUG_ON(tnetv107x_fb_color_mode_to_bpp(fbdev->color_mode) != bpp);

	fix->visual = tnetv107x_fb_color_mode_to_visual(fbdev->color_mode);

	fix->accel		= 0;
	fix->line_length	= var->xres_virtual * bpp / 8;
	fix->xpanstep		= 0;
	fix->ypanstep		= 1;
	fix->ywrapstep		= 0;

	return changed;
}

static int tnetv107x_fb_check_var(struct fb_var_screeninfo *var,
		struct fb_info *fbi)
{
	unsigned long	max_frame_size;
	unsigned long	line_size;
	struct tnetv107x_fb_device *fbdev = fbi->par;
	enum tnetv107x_fb_color_format color_mode;
	u32 bpp;

	if (var->nonstd)
		return -EINVAL;

	color_mode = tnetv107x_fb_bpp_to_color_mode(var->bits_per_pixel);
	if (color_mode < 0)
		return -EINVAL;

	bpp = var->bits_per_pixel = tnetv107x_fb_color_mode_to_bpp(color_mode);

	if (var->xres_virtual < var->xres)
		var->xres_virtual = var->xres;
	if (var->yres_virtual < var->yres)
		var->yres_virtual = var->yres;
	max_frame_size = fbdev->vram_size;
	line_size = var->xres_virtual * bpp / 8;
	if (line_size * var->yres_virtual > max_frame_size) {
		/* Try to keep yres_virtual first */
		line_size = max_frame_size / var->yres_virtual;
		var->xres_virtual = line_size * 8 / bpp;
		if (var->xres_virtual < var->xres) {
			/* Still doesn't fit. Shrink yres_virtual too */
			var->xres_virtual = var->xres;
			line_size = var->xres * bpp / 8;
			var->yres_virtual = max_frame_size / line_size;
		}
	}
	if (var->xres + var->xoffset > var->xres_virtual)
		var->xoffset = var->xres_virtual - var->xres;
	if (var->yres + var->yoffset > var->yres_virtual)
		var->yoffset = var->yres_virtual - var->yres;
	line_size = var->xres * bpp / 8;

#define set_var(var, field, offs, len)		\
	do {					\
		var->field.offset = offs;	\
		var->field.length = len;	\
		var->field.msb_right = 0;	\
	} while (0)

	switch (color_mode) {
		case FB_COLOR_RGB444:
			set_var(var, red,    8,  4);
			set_var(var, green,  4,  4);
			set_var(var, blue,   0,  4);
			break;

		case FB_COLOR_RGB565:
			set_var(var, red,   11,  5);
			set_var(var, green,  5,  6);
			set_var(var, blue,   0,  5);
			break;

		case FB_COLOR_RGB888:
			set_var(var, red,   16,  8);
			set_var(var, green,  8,  8);
			set_var(var, blue,   0,  8);
			break;

		default: /* CLUT modes */
			break;
	}

	var->grayscale		= 0;
	var->vmode		= FB_VMODE_NONINTERLACED;
	var->sync		= 0;

	return 0;
}

/*
 * Set new x,y offsets in the virtual display for the visible area and switch
 * to the new mode.
 */
static int tnetv107x_fb_pan_display(struct fb_var_screeninfo *var,
		struct fb_info *fbi)
{
	int ret = 0;
	struct fb_var_screeninfo new_var;
	struct tnetv107x_fb_device *fbdev = fbi->par;

	mutex_lock(&fbdev->rqueue_mutex);
	if (var->xoffset != fbi->var.xoffset ||
			var->yoffset != fbi->var.yoffset) {
		memcpy(&new_var, &fbi->var, sizeof(new_var));
		new_var.xoffset = var->xoffset;
		new_var.yoffset = var->yoffset;
		if (tnetv107x_fb_check_var(&new_var, fbi))
			ret = -EINVAL;
		else {
			memcpy(&fbi->var, &new_var, sizeof(new_var));
			ret = tnetv107x_lcdc_pan_display(fbdev);
		}
	}
	mutex_unlock(&fbdev->rqueue_mutex);

	return ret;
}

/*
 * Switch to a new mode. The parameters for it has been check already by
 * tnetv107x_fb_check_var.
 */
static int tnetv107x_fb_set_par(struct fb_info *fbi)
{
	int ret = 0;
	struct tnetv107x_fb_device *fbdev = fbi->par;
	bool changed = false;

	mutex_lock(&fbdev->rqueue_mutex);
	
	changed = tnetv107x_fb_set_fix(fbi);

	if (!changed) {
#define is_changed(field) fbi->var.field != fbdev->cur_var.field
		/* check if any of the mode dependencies have changed */
		if (is_changed(pixclock)	||
		    is_changed(xres)		||
		    is_changed(yres)		||
		    is_changed(hsync_len)	||
		    is_changed(left_margin)	||
		    is_changed(right_margin)	||
		    is_changed(vsync_len)	||
		    is_changed(upper_margin)	||
		    is_changed(lower_margin))
			changed = true;
	}

	if (changed) {
		ret = tnetv107x_lcdc_change_mode(fbdev);
		fbdev->cur_var = fbi->var;
	}

	mutex_unlock(&fbdev->rqueue_mutex);

	return ret;
}

static struct fb_ops tnetv107x_fb_ops = {
	.owner		= THIS_MODULE,
	.fb_open        = tnetv107x_fb_open,
	.fb_release     = tnetv107x_fb_release,
	.fb_setcolreg	= tnetv107x_fb_setcolreg,
	.fb_setcmap	= tnetv107x_fb_setcmap,
	.fb_blank       = tnetv107x_fb_blank,
	.fb_check_var	= tnetv107x_fb_check_var,
	.fb_set_par	= tnetv107x_fb_set_par,
	.fb_pan_display = tnetv107x_fb_pan_display,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
};



/*****************************************************************************/
/*  Framebuffer Layer initialization and cleanup routines                    */
/*****************************************************************************/

static void tnetv107x_fb_cleanup_vram(struct tnetv107x_fb_device *fbdev) {
	if (fbdev->vram_virt) {
		vunmap(fbdev->vram_virt);
		fbdev->vram_virt = NULL;
	}

	if (fbdev->vram_alloc) {
		dma_free_writecombine(fbdev->dev, fbdev->vram_size,
				fbdev->vram_virt, fbdev->vram_phys);
		fbdev->vram_alloc = 0;
	}

	if (fbdev->vram_res) {
		release_resource(fbdev->vram_res);
		kfree(fbdev->vram_res);
	}
}

static int tnetv107x_fb_init_vram(struct tnetv107x_fb_device *fbdev) {
	struct resource		*res;
	int			bpp;
	struct tnetv107x_lcd_panel	*panel = fbdev->panel;
	struct vm_struct	*kvma;
	struct vm_area_struct	vma;
	pgprot_t		pgprot;
	unsigned long		vaddr;


	res = platform_get_resource_byname(fbdev->pdev,
			IORESOURCE_MEM, "vram");
	if (!res) goto need_alloc;

	fbdev->vram_res = request_mem_region(res->start,
			res_size(res), fbdev->pdev->name);
	if (!fbdev->vram_res) goto need_alloc;
	fbdev->vram_phys = fbdev->vram_res->start;
	fbdev->vram_size = res_size(fbdev->vram_res);
	goto map_virt;

need_alloc:
	/* Platform device did not include a VRAM area.
	 * We try allocating one here */
	bpp = panel->bpp;
	if (bpp == 12) bpp = 16;

	fbdev->vram_size = (panel->x_res * panel->y_res * bpp);
	fbdev->vram_size = PAGE_ALIGN(fbdev->vram_size/8);
	fbdev->vram_size = fbdev->vram_size * fbdev->data.virt_screens;

	vaddr = (unsigned long)dma_alloc_writecombine(fbdev->dev,
			fbdev->vram_size, &fbdev->vram_phys, GFP_KERNEL);
	if (!vaddr) {
		dev_err(fbdev->dev, "unable to allocate framebuffer memory\n");
		return -ENOMEM;
	}

	fbdev->vram_alloc = 1;

map_virt:
	/* Physical memory allocated, create a write combined VMA map */
	kvma = get_vm_area(fbdev->vram_size, VM_IOREMAP);
	if (!kvma) {
		dev_err(fbdev->dev, "cannot get kernel vm area\n");
		tnetv107x_fb_cleanup_vram(fbdev);
		return -ENOMEM;
	}
	vma.vm_mm = &init_mm;

	vaddr = (unsigned long)kvma->addr;
	vma.vm_start = vaddr;
	vma.vm_end = vaddr + fbdev->vram_size;

	pgprot = pgprot_writecombine(pgprot_kernel);
	if (io_remap_pfn_range(&vma, vaddr,
			   fbdev->vram_phys >> PAGE_SHIFT,
			   fbdev->vram_size, pgprot) < 0) {
		dev_err(fbdev->dev, "kernel mmap for FB memory failed\n");
		tnetv107x_fb_cleanup_vram(fbdev);
		return -EAGAIN;
	}

	fbdev->vram_virt = (void *)vaddr;

	return 0;
}

static void tnetv107x_fb_cleanup_info(struct tnetv107x_fb_device *fbdev,
		struct fb_info *fbi)
{
	fb_dealloc_cmap(&fbi->cmap);
}

/*
 * Initialize system fb_info object and set the default video mode.
 * The frame buffer memory already allocated by lcddma_init
 */
static int tnetv107x_fb_init_info(struct tnetv107x_fb_device *fbdev,
		struct fb_info *info)
{
	struct fb_var_screeninfo	*var = &info->var;
	struct fb_fix_screeninfo	*fix = &info->fix;
	struct tnetv107x_lcd_panel		*panel = fbdev->panel;
	int				ret = 0, bpl;

	info->fbops = &tnetv107x_fb_ops;
	info->flags = FBINFO_FLAG_DEFAULT;

	strncpy(fix->id, "tnetv107x-fb", sizeof(fix->id));

	info->screen_base	= (char __iomem *)fbdev->vram_virt;
	fix->smem_start		= fbdev->vram_phys;
	fix->smem_len		= fbdev->vram_size;

	info->pseudo_palette = fbdev->pseudo_palette;

	var->accel_flags  = 0;
	var->bits_per_pixel = fbdev->panel->bpp;
	var->xres = fbdev->panel->x_res;
	var->yres = fbdev->panel->y_res;
	var->xres_virtual = fbdev->panel->x_res;

	bpl = (fbdev->panel->bpp == 12) ? 16 : fbdev->panel->bpp;
	bpl *= var->xres; bpl /= 8;
	var->yres_virtual = fbdev->vram_size/bpl;

	var->xres		= panel->x_res;
	var->yres		= panel->y_res;
	var->height		= panel->height ? panel->height : -1;
	var->width		= panel->width  ? panel->width  : -1;
	var->pixclock		= panel->pixel_clock;
	var->left_margin	= panel->hfp;
	var->right_margin	= panel->hbp;
	var->upper_margin	= panel->vfp;
	var->lower_margin	= panel->vbp;
	var->hsync_len		= panel->hsw;
	var->vsync_len		= panel->vsw;
	var->grayscale		= 0;


	tnetv107x_fb_check_var(var, info);
	tnetv107x_fb_set_fix(info);

	ret = fb_alloc_cmap(&info->cmap, 16, 0);
	if (ret != 0)
		dev_err(fbdev->dev, "unable to allocate color map memory\n");

	return ret;
}

static void tnetv107x_fb_cleanup(struct tnetv107x_fb_device *fbdev,
					int state)
{
	switch (state) {
		case FB_ACTIVE:
			tnetv107x_lcdc_suspend(fbdev);
		case FB_SUSPENDED:
			unregister_framebuffer(fbdev->fb_info);
		case 5:
			tnetv107x_fb_cleanup_info(fbdev, fbdev->fb_info);
		case 4:
			tnetv107x_lcdc_cleanup(fbdev);
		case 3:
			tnetv107x_fb_cleanup_vram(fbdev);
		case 2:
			if (fbdev->panel->cleanup)
				fbdev->panel->cleanup(fbdev->panel, fbdev);
		case 1:
			dev_set_drvdata(fbdev->dev, NULL);
			framebuffer_release(fbdev->fb_info);
		case 0:
			/* nothing to free */
			break;
		default:
			BUG();
	}
}

static int tnetv107x_fb_init(struct platform_device *pdev) {
	struct device		*dev = &pdev->dev;
	struct fb_info		*fbi;
	int			init_state;
	int			ret = 0;
	struct tnetv107x_fb_device *fbdev = NULL;
	struct tnetv107x_lcd_panel *panel = NULL;
	struct tnetv107x_fb_data   *data  = dev->platform_data;

	init_state = 0;

	if (!data) {
		dev_err(&pdev->dev, "could not find platform data\n");
		ret = -EINVAL;
		goto cleanup;
	}

	panel = tnetv107x_panel_find(data->panel_name);
	if (!panel) {
		dev_err(&pdev->dev, "could not find panel %s\n",
				data->panel_name);
		ret = -EINVAL;
		goto cleanup;
	}


	fbi = framebuffer_alloc(sizeof(struct tnetv107x_fb_device),
				&pdev->dev);
	if (!fbi) {
		dev_err(&pdev->dev, "unable to allocate framebuffer\n");
		ret = -ENOMEM;
		goto cleanup;
	}

	fbdev = fbi->par;
	fbdev->fb_info = fbi;
	fbdev->dev = dev;
	fbdev->pdev = pdev;
	fbdev->panel = panel;
	fbdev->data = *data;
	platform_set_drvdata(pdev, fbdev);

	init_state++;		/* 1 */

	mutex_init(&fbdev->rqueue_mutex);

	if (panel->init) {
		ret = panel->init(fbdev->panel, fbdev);
		if (ret)
			goto cleanup;
	}

	dev_dbg(dev, "configured %s panel\n", fbdev->panel->name);

	init_state++;		/* 2 */

	ret = tnetv107x_fb_init_vram(fbdev);
	if (ret)
		goto cleanup;
	init_state++;		/* 3 */

	dev_dbg(dev, "vram %lukB @ %08lx (virt %08lx)\n",
			(unsigned long)fbdev->vram_size >> 10,
			(unsigned long)fbdev->vram_phys,
			(unsigned long)fbdev->vram_virt);

	ret = tnetv107x_lcdc_init(fbdev);
	if (ret)
		goto cleanup;
	init_state++;		/* 4 */

	ret = tnetv107x_fb_init_info(fbdev, fbi);
	if (ret)
		goto cleanup;
	init_state++;		/* 5 */

	ret = register_framebuffer(fbdev->fb_info);
	if (ret != 0) {
		dev_err(fbdev->dev, "failed to register framebuffer\n");
		goto cleanup;
	}

	fbdev->state = init_state = FB_SUSPENDED;

	ret = tnetv107x_lcdc_resume(fbdev);
	if (ret != 0) {
		dev_err(fbdev->dev, "failed to resume controller\n");
		goto cleanup;
	}

	fbdev->state = init_state = FB_ACTIVE;

	return 0;

cleanup:
	tnetv107x_fb_cleanup(fbdev, init_state);

	return ret;
}


/*****************************************************************************/
/*  Framebuffer Platform Device                                              */
/*****************************************************************************/

static int tnetv107x_fb_probe(struct platform_device *pdev) {
	return tnetv107x_fb_init(pdev);
}

static int tnetv107x_fb_remove(struct platform_device *pdev) {
	struct tnetv107x_fb_device *fbdev = platform_get_drvdata(pdev);
	enum tnetv107x_fb_state saved_state = fbdev->state;

	fbdev->state = FB_DISABLED;
	tnetv107x_fb_cleanup(fbdev, saved_state);

	return 0;
}

static int
tnetv107x_fb_suspend(struct platform_device *pdev, pm_message_t mesg) {
	struct tnetv107x_fb_device *fbdev = platform_get_drvdata(pdev);

	tnetv107x_fb_blank(VESA_POWERDOWN, fbdev->fb_info);

	return 0;
}

static int
tnetv107x_fb_resume(struct platform_device *pdev) {
	struct tnetv107x_fb_device *fbdev = platform_get_drvdata(pdev);

	tnetv107x_fb_blank(VESA_NO_BLANKING, fbdev->fb_info);
	return 0;
}

static struct platform_driver tnetv107x_fb_driver = {
	.probe		= tnetv107x_fb_probe,
	.remove		= tnetv107x_fb_remove,
	.suspend	= tnetv107x_fb_suspend,
	.resume		= tnetv107x_fb_resume,
	.driver		= {
		.name	= "tnetv107x-fb",
		.owner	= THIS_MODULE,
	},
};

static int __init tnetv107x_fb_drv_init(void) {
	return platform_driver_register(&tnetv107x_fb_driver);
}

static void __exit tnetv107x_fb_drv_exit(void) {
	platform_driver_unregister(&tnetv107x_fb_driver);
}

module_init(tnetv107x_fb_drv_init);
module_exit(tnetv107x_fb_drv_exit);

MODULE_DESCRIPTION("TNETV107X LCD Framebuffer");
MODULE_AUTHOR("Cyril Chemparathy <cyril@ti.com>");
MODULE_LICENSE("GPL");

EXPORT_SYMBOL(tnetv107x_fb_sync_notifier_register);
EXPORT_SYMBOL(tnetv107x_fb_sync_notifier_unregister);
