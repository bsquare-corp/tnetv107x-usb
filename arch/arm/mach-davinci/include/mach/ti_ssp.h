/*
 * Sequencer Serial Port (SSP) driver for Texas Instruments' SoCs
 *
 * Copyright (C) 2010 Texas Instruments Inc
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

#ifndef __TI_SSP_H__
#define __TI_SSP_H__

struct ti_ssp_device {
	u32		id;
	const char	*name;
	int		port;
	unsigned long	min_clock;
	unsigned long	max_clock;
	unsigned long	iosel; /* Note 1 below */
	unsigned long	config;
	void		*priv;
};

struct ti_ssp_data {
	int			num_devices;
	struct ti_ssp_device	*devices;
	int			(*select)(u32 id, int enable);
};

/* Sequencer port IO pin configuration bits.
 * These do not correlate 1-1 with the hardware
 *
 * Note 1: The iosel field combines iosel1 and iosel2,
 * and is therefore not a direct map to register space.
 * It is best to use the macros below to construct iosel
 * values.
 *
 * least significant 16 bits = iosel1,
 * most significant 16 bits = iosel2 */

#define SSP_IN			0x0000
#define SSP_DATA		0x0001
#define SSP_CLOCK		0x0002
#define SSP_CHIPSEL		0x0003
#define SSP_OUT			0x0004
#define SSP_PIN_SEL(pin, v)	((v)<<((pin)*3))
#define SSP_INPUT_SEL(pin)	((pin) << 16)

/* Sequencer port config bits */
#define SSP_EARLY_DIN		(1 << 8)
#define SSP_DELAY_DOUT		(1 << 9)

/* Sequence map definitions */
#define SSP_CLK_HIGH		(1 << 0)
#define SSP_CLK_LOW		(0 << 0)
#define SSP_DATA_HIGH		(1 << 1)
#define SSP_DATA_LOW		(0 << 1)
#define SSP_CS_HIGH		(1 << 2)
#define SSP_CS_LOW		(0 << 2)
#define SSP_IN_MODE		(0 << 3)
#define SSP_OUT_MODE		(1 << 3)
#define SSP_DATA_REG		(1 << 4)
#define SSP_ADDR_REG		(0 << 4)
#define SSP_OPCODE_DIRECT	((0x0) << 5)
#define SSP_OPCODE_TOGGLE	((0x1) << 5)
#define SSP_OPCODE_SHIFT	((0x2) << 5)
#define SSP_OPCODE_BRANCH0	((0x4) << 5)
#define SSP_OPCODE_BRANCH1	((0x5) << 5)
#define SSP_OPCODE_BRANCH	((0x6) << 5)
#define SSP_OPCODE_STOP		((0x7) << 5)
#define SSP_BRANCH(addr)	((addr)<<8)
#define SSP_COUNT(cycles)	((cycles)<<8)

struct ti_ssp_device *ti_ssp_open(const char *name);
int ti_ssp_close(struct ti_ssp_device *dev);
int ti_ssp_dumpregs(struct ti_ssp_device *dev);
int ti_ssp_raw_read(struct ti_ssp_device *dev);
int ti_ssp_raw_write(struct ti_ssp_device *dev, u32 val);
int ti_ssp_load(struct ti_ssp_device *dev, int offs, u32* prog, int len);
int ti_ssp_run(struct ti_ssp_device *dev, u32 pc, u32 input, u32 *output);

#endif /* __TI_SSP_H__ */
