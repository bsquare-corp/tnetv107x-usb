/*
 * Texas Instruments 3-Port Ethernet Switch Address Lookup Engine
 *
 * Copyright (C) 2010 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/ctype.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/io.h>

#include "cpsw_ale.h"

#define BITMASK(bits)		(BIT(bits) - 1)
#define ADDR_FMT_STR		"%02x:%02x:%02x:%02x:%02x:%02x"
#define ADDR_FMT_ARGS(addr)	(addr)[0], (addr)[1], (addr)[2], \
				(addr)[3], (addr)[4], (addr)[5]
#define ALE_ENTRY_BITS		68
#define ALE_ENTRY_WORDS		DIV_ROUND_UP(ALE_ENTRY_BITS, 32)

/* ALE Registers */
#define ALE_IDVER		0x00
#define ALE_CONTROL		0x08
#define ALE_PRESCALE		0x10
#define ALE_UNKNOWNVLAN		0x18
#define ALE_TABLE_CONTROL	0x20
#define ALE_TABLE		0x34
#define ALE_PORTCTL		0x40

#define ALE_TABLE_WRITE		BIT(31)

#define ALE_TYPE_FREE			0
#define ALE_TYPE_ADDR			1
#define ALE_TYPE_VLAN			2
#define ALE_TYPE_VLAN_ADDR		3

#define ALE_UCAST_PERSISTANT		0
#define ALE_UCAST_UNTOUCHED		1
#define ALE_UCAST_OUI			2
#define ALE_UCAST_TOUCHED		3

#define ALE_MCAST_FWD			0
#define ALE_MCAST_BLOCK_LEARN_FWD	1
#define ALE_MCAST_FWD_LEARN		2
#define ALE_MCAST_FWD_2			3

/* the following remap params into members of cpsw_ale */
#define ale_regs	params.ale_regs
#define ale_entries	params.ale_entries
#define ale_ports	params.ale_ports

static inline int cpsw_ale_get_field(u32 *ale_entry, u32 start, u32 bits)
{
	int idx, len = ALE_ENTRY_BITS;

	WARN_ON(bits > 30 || bits + start > len);
	idx    = start / 32;
	start -= idx * 32;
	idx    = 2 - idx; /* flip */
	WARN_ON(start + bits > 32); /* straddles word boundaries */
	return (ale_entry[idx] >> start) & BITMASK(bits);
}

static inline void cpsw_ale_set_field(u32 *ale_entry, u32 start, u32 bits,
				      u32 value)
{
	int idx, len = ALE_ENTRY_BITS;

	WARN_ON(bits > 30 || bits + start > len);
	value &= BITMASK(bits);
	idx    = start / 32;
	start -= idx * 32;
	idx    = 2 - idx; /* flip */
	WARN_ON(start + bits > 32); /* straddles word boundaries */
	ale_entry[idx] &= ~(BITMASK(bits) << start);
	ale_entry[idx] |=  (value << start);
}

#define DEFINE_ALE_FIELD(name, start, bits)				\
static inline int cpsw_ale_get_##name(u32 *ale_entry)			\
{									\
	return cpsw_ale_get_field(ale_entry, start, bits);		\
}									\
static inline void cpsw_ale_set_##name(u32 *ale_entry, u32 value)	\
{									\
	cpsw_ale_set_field(ale_entry, start, bits, value);		\
}

DEFINE_ALE_FIELD(entry_type,		60,	2)
DEFINE_ALE_FIELD(vlan_id,		48,	12)
DEFINE_ALE_FIELD(mcast_state,		62,	2)
DEFINE_ALE_FIELD(port_mask,		64,	3)
DEFINE_ALE_FIELD(super,			67,	1)
DEFINE_ALE_FIELD(ucast_type,		66,	2)
DEFINE_ALE_FIELD(port_num,		64,	2)
DEFINE_ALE_FIELD(blocked,		63,	1)
DEFINE_ALE_FIELD(secure,		62,	1)
DEFINE_ALE_FIELD(vlan_untag_force,	24,	3)
DEFINE_ALE_FIELD(vlan_reg_mcast,	16,	3)
DEFINE_ALE_FIELD(vlan_unreg_mcast,	8,	3)
DEFINE_ALE_FIELD(vlan_member_list,	0,	3)
DEFINE_ALE_FIELD(mcast,			47,	1)

/* The MAC address field in the ALE entry cannot be macroized as above */
static inline void cpsw_ale_get_addr(u32 *ale_entry, u8 *addr)
{
	int i;

	for (i = 0; i < 6; i++)
		addr[i] = cpsw_ale_get_field(ale_entry, 40 - 8*i, 8);
}

static inline void cpsw_ale_set_addr(u32 *ale_entry, u8 *addr)
{
	int i;

	for (i = 0; i < 6; i++)
		cpsw_ale_set_field(ale_entry, 40 - 8*i, 8, addr[i]);
}

static int cpsw_ale_read(struct cpsw_ale *ale, int idx, u32 *ale_entry)
{
	int i;

	WARN_ON(idx > ale->ale_entries);

	__raw_writel(idx, ale->ale_regs + ALE_TABLE_CONTROL);

	for (i = 0; i < ALE_ENTRY_WORDS; i++)
		ale_entry[i] = __raw_readl(ale->ale_regs + ALE_TABLE + 4 * i);

	return idx;
}

static int cpsw_ale_write(struct cpsw_ale *ale, int idx, u32 *ale_entry)
{
	int i;

	WARN_ON(idx > ale->ale_entries);

	for (i = 0; i < ALE_ENTRY_WORDS; i++)
		__raw_writel(ale_entry[i], ale->ale_regs + ALE_TABLE + 4 * i);

	__raw_writel(idx | ALE_TABLE_WRITE, ale->ale_regs + ALE_TABLE_CONTROL);

	return idx;
}

static int cpsw_ale_match_addr(struct cpsw_ale *ale, u8* addr)
{
	u32 ale_entry[ALE_ENTRY_WORDS];
	int type, idx;

	for (idx = 0; idx < ale->ale_entries; idx++) {
		u8 entry_addr[6];

		cpsw_ale_read(ale, idx, ale_entry);
		type = cpsw_ale_get_entry_type(ale_entry);
		if (type != ALE_TYPE_ADDR && type != ALE_TYPE_VLAN_ADDR)
			continue;
		cpsw_ale_get_addr(ale_entry, entry_addr);
		if (memcmp(entry_addr, addr, 6) == 0)
			return idx;
	}
	return -ENOENT;
}

static int cpsw_ale_match_vlan(struct cpsw_ale *ale, u16 vid)
{
	u32 ale_entry[ALE_ENTRY_WORDS];
	int type, idx;

	for (idx = 0; idx < ale->ale_entries; idx++) {
		cpsw_ale_read(ale, idx, ale_entry);
		type = cpsw_ale_get_entry_type(ale_entry);
		if (type != ALE_TYPE_VLAN)
			continue;
		if (cpsw_ale_get_vlan_id(ale_entry) == vid)
			return idx;
	}
	return -ENOENT;
}

static int cpsw_ale_match_free(struct cpsw_ale *ale)
{
	u32 ale_entry[ALE_ENTRY_WORDS];
	int type, idx;

	for (idx = 0; idx < ale->ale_entries; idx++) {
		cpsw_ale_read(ale, idx, ale_entry);
		type = cpsw_ale_get_entry_type(ale_entry);
		if (type == ALE_TYPE_FREE)
			return idx;
	}
	return -ENOENT;
}

static int cpsw_ale_find_ageable(struct cpsw_ale *ale)
{
	u32 ale_entry[ALE_ENTRY_WORDS];
	int type, idx;

	for (idx = 0; idx < ale->ale_entries; idx++) {
		cpsw_ale_read(ale, idx, ale_entry);
		type = cpsw_ale_get_entry_type(ale_entry);
		if (type != ALE_TYPE_ADDR && type != ALE_TYPE_VLAN_ADDR)
			continue;
		if (cpsw_ale_get_mcast(ale_entry))
			continue;
		type = cpsw_ale_get_ucast_type(ale_entry);
		if (type != ALE_UCAST_PERSISTANT &&
		    type != ALE_UCAST_OUI)
			return idx;
	}
	return -ENOENT;
}

static void cpsw_ale_flush_mcast(struct cpsw_ale *ale, u32 *ale_entry,
				 int port_mask)
{
	int mask;

	mask = cpsw_ale_get_port_mask(ale_entry);
	if ((mask & port_mask) == 0)
		return; /* ports dont intersect, not interested */
	mask &= ~port_mask;

	/* free if no ports left */
	if (!mask)
		cpsw_ale_set_entry_type(ale_entry, ALE_TYPE_FREE);
	else
		cpsw_ale_set_port_mask(ale_entry, mask);
}

static void cpsw_ale_flush_ucast(struct cpsw_ale *ale, u32 *ale_entry,
				 int port_mask)
{
	int port;

	port = cpsw_ale_get_port_num(ale_entry);
	if ((BIT(port) & port_mask) == 0)
		return; /* ports dont intersect, not interested */
	cpsw_ale_set_entry_type(ale_entry, ALE_TYPE_FREE);
}

int cpsw_ale_flush(struct cpsw_ale *ale, int port_mask)
{
	u32 ale_entry[ALE_ENTRY_WORDS];
	int ret, idx;

	for (idx = 0; idx < ale->ale_entries; idx++) {
		cpsw_ale_read(ale, idx, ale_entry);
		ret = cpsw_ale_get_entry_type(ale_entry);
		if (ret != ALE_TYPE_ADDR && ret != ALE_TYPE_VLAN_ADDR)
			continue;

		if (cpsw_ale_get_mcast(ale_entry))
			cpsw_ale_flush_mcast(ale, ale_entry, port_mask);
		else
			cpsw_ale_flush_ucast(ale, ale_entry, port_mask);

		cpsw_ale_write(ale, idx, ale_entry);
	}
	return 0;
}

static const char *str_mcast_state[] = {"f", "blf", "lf", "f"};

static int cpsw_ale_dump_mcast(u32 *ale_entry, char *buf, int len)
{
	int outlen = 0;
	int mcast_state = cpsw_ale_get_mcast_state(ale_entry);
	int port_mask   = cpsw_ale_get_port_mask(ale_entry);
	int super       = cpsw_ale_get_super(ale_entry);

	outlen += snprintf(buf + outlen, len - outlen,
			   "mcstate: %s, ", str_mcast_state[mcast_state]);
	outlen += snprintf(buf + outlen, len - outlen,
			   "portmask: %x, ", port_mask);
	outlen += snprintf(buf + outlen, len - outlen,
			   "super: %s\n", super ? "y" : "n");
	return outlen;
}

static const char *str_ucast_type[] = {"persistant", "untouched", "oui",
				       "touched"};

static int cpsw_ale_dump_ucast(u32 *ale_entry, char *buf, int len)
{
	int outlen = 0;
	int ucast_type  = cpsw_ale_get_ucast_type(ale_entry);
	int port_num    = cpsw_ale_get_port_num(ale_entry);
	int secure      = cpsw_ale_get_secure(ale_entry);
	int blocked     = cpsw_ale_get_blocked(ale_entry);

	outlen += snprintf(buf + outlen, len - outlen,
			   "uctype: %s, ", str_ucast_type[ucast_type]);
	outlen += snprintf(buf + outlen, len - outlen,
			   "port: %d, ", port_num);
	outlen += snprintf(buf + outlen, len - outlen,
			   "secure: %s, ", secure ? "y" : "n");
	outlen += snprintf(buf + outlen, len - outlen,
			   "blocked: %s\n", blocked ? "y" : "n");
	return outlen;
}

static int cpsw_ale_dump_vlan(u32 *ale_entry, char *buf, int len)
{
	int outlen = 0;
	int untag_force = cpsw_ale_get_vlan_untag_force(ale_entry);
	int reg_mcast   = cpsw_ale_get_vlan_reg_mcast(ale_entry);
	int unreg_mcast = cpsw_ale_get_vlan_unreg_mcast(ale_entry);
	int member_list = cpsw_ale_get_vlan_member_list(ale_entry);

	outlen += snprintf(buf + outlen, len - outlen,
			   "vlanuntag: %x, ", untag_force);
	outlen += snprintf(buf + outlen, len - outlen,
			   "vlanregmcast: %x, ", reg_mcast);
	outlen += snprintf(buf + outlen, len - outlen,
			   "vlanunregmcast: %x, ", unreg_mcast);
	outlen += snprintf(buf + outlen, len - outlen,
			   "vlanmemberlist: %x\n", member_list);
	return outlen;
}

static const char *str_type[] = {"free", "addr", "vlan", "vlanaddr"};

static int cpsw_ale_dump_entry(int idx, u32 *ale_entry, char *buf, int len)
{
	int type, outlen = 0;

	type = cpsw_ale_get_entry_type(ale_entry);
	if (type == ALE_TYPE_FREE)
		return outlen;

	if (idx >= 0) {
		outlen += snprintf(buf + outlen, len - outlen,
				   "index: %d, ", idx);
	}

	outlen += snprintf(buf + outlen, len - outlen, "raw: %08x %08x %08x, ",
			   ale_entry[0], ale_entry[1], ale_entry[2]);

	outlen += snprintf(buf + outlen, len - outlen,
			   "type: %s, ", str_type[type]);

	if (type == ALE_TYPE_VLAN || type == ALE_TYPE_VLAN_ADDR) {
		outlen += snprintf(buf + outlen, len - outlen, "vlan: %d, ",
				   cpsw_ale_get_vlan_id(ale_entry));
	}

	if (type == ALE_TYPE_ADDR || type == ALE_TYPE_VLAN_ADDR) {
		u8 addr[6];

		cpsw_ale_get_addr(ale_entry, addr);
		outlen += snprintf(buf + outlen, len - outlen,
				   "addr: " ADDR_FMT_STR ", ",
				   ADDR_FMT_ARGS(addr));

		if (cpsw_ale_get_mcast(ale_entry)) {
			outlen += cpsw_ale_dump_mcast(ale_entry, buf + outlen,
						      len - outlen);
		} else {
			outlen += cpsw_ale_dump_ucast(ale_entry, buf + outlen,
						      len - outlen);
		}
	} else { /* type == ALE_TYPE_VLAN */
		outlen += cpsw_ale_dump_vlan(ale_entry, buf + outlen,
					     len - outlen);
	}

	return outlen;
}

static int cpsw_ale_parse_entry(int *idx, u32 *ale_entry, const char *buf,
				int len)
{
	const char *end = buf + len;
	char name[32], value[32];
	int taglen, outlen;

	memset(ale_entry, 0, ALE_ENTRY_WORDS * sizeof(*ale_entry));

	while (buf < end) {
		taglen = strcspn(buf, ":") + 1;
		for (outlen = 0; taglen; buf++, taglen--)
			if (isalpha(*buf))
				name[outlen++] = tolower(*buf);
		name[outlen] = '\0';

		if (buf >= end)
			return -EINVAL;

		taglen = strcspn(buf, ",") + 1;
		for (outlen = 0; taglen; buf++, taglen--)
			if (isalnum(*buf))
				value[outlen++] = tolower(*buf);
		value[outlen] = '\0';

		if (strcmp(name, "index") == 0) {
			*idx = simple_strtoul(value, NULL, 10);
		}
		else if (strcmp(name, "raw") == 0) {
			/* ignore */
		}
		else if (strcmp(name, "type") == 0) {
			int type;
			for (type = 0; type < ARRAY_SIZE(str_type); type++)
				if (strcmp(value, str_type[type]) == 0)
					break;
			if (type >= ARRAY_SIZE(str_type))
				return -EINVAL;
			cpsw_ale_set_entry_type(ale_entry, type);
		}
		else if (strcmp(name, "addr") == 0) {
			u8 addr[6], i;
			int h;

			for (i = 0; i < strlen(value); i++) {
				h = hex_to_bin(value[i]);
				if (h < 0)
					return -EINVAL;
				addr[i / 2] = addr[i / 2] << 4 | h;
			}
			cpsw_ale_set_addr(ale_entry, addr);
		}
		else if (strcmp(name, "vlan") == 0) {
			int vlan_id;
			vlan_id = simple_strtoul(value, NULL, 10);
			cpsw_ale_set_vlan_id(ale_entry, vlan_id);
		}
		else if (strcmp(name, "mcstate") == 0) {
			int mc;
			for (mc = 0; mc < ARRAY_SIZE(str_mcast_state); mc++)
				if (strcmp(value, str_mcast_state[mc]) == 0)
					break;
			if (mc >= ARRAY_SIZE(str_mcast_state))
				return -EINVAL;
			cpsw_ale_set_mcast_state(ale_entry, mc);
		}
		else if (strcmp(name, "portmask") == 0) {
			int port_mask;
			port_mask = simple_strtoul(value, NULL, 16);
			cpsw_ale_set_port_mask(ale_entry, port_mask);
		}
		else if (strcmp(name, "super") == 0) {
			int super = 0;
			if (strcmp(value, "y") == 0)
				super = 1;
			else if (strcmp(value, "n") != 0)
				return -EINVAL;
			cpsw_ale_set_super(ale_entry, super);
		}
		else if (strcmp(name, "uctype") == 0) {
			int uc;
			for (uc = 0; uc < ARRAY_SIZE(str_ucast_type); uc++)
				if (strcmp(value, str_ucast_type[uc]) == 0)
					break;
			if (uc >= ARRAY_SIZE(str_ucast_type))
				return -EINVAL;
			cpsw_ale_set_ucast_type(ale_entry, uc);
		}
		else if (strcmp(name, "port") == 0) {
			int port;
			port = simple_strtoul(value, NULL, 10);
			cpsw_ale_set_port_num(ale_entry, port);
		}
		else if (strcmp(name, "secure") == 0) {
			int secure = 0;
			if (strcmp(value, "y") == 0)
				secure = 1;
			else if (strcmp(value, "n") != 0)
				return -EINVAL;
			cpsw_ale_set_secure(ale_entry, secure);
		}
		else if (strcmp(name, "blocked") == 0) {
			int blocked = 0;
			if (strcmp(value, "y") == 0)
				blocked = 1;
			else if (strcmp(value, "n") != 0)
				return -EINVAL;
			cpsw_ale_set_blocked(ale_entry, blocked);
		}
		else if (strcmp(name, "vlanuntag") == 0) {
			int untag_force;
			untag_force = simple_strtoul(value, NULL, 16);
			cpsw_ale_set_vlan_untag_force(ale_entry, untag_force);
		}
		else if (strcmp(name, "vlanregmcast") == 0) {
			int reg_mcast;
			reg_mcast = simple_strtoul(value, NULL, 16);
			cpsw_ale_set_vlan_reg_mcast(ale_entry, reg_mcast);
		}
		else if (strcmp(name, "vlanunregmcast") == 0) {
			int unreg_mcast;
			unreg_mcast = simple_strtoul(value, NULL, 16);
			cpsw_ale_set_vlan_unreg_mcast(ale_entry, unreg_mcast);
		}
		else if (strcmp(name, "vlanmemberlist") == 0) {
			int member_list;
			member_list = simple_strtoul(value, NULL, 16);
			cpsw_ale_set_vlan_member_list(ale_entry, member_list);
		}
		else
			return -EINVAL;
	}
	return 0;
}

int cpsw_ale_add_ucast(struct cpsw_ale *ale, u8 *addr, int port, int flags)
{
	u32 ale_entry[ALE_ENTRY_WORDS] = {0, 0, 0};
	int idx;

	cpsw_ale_set_entry_type(ale_entry, ALE_TYPE_ADDR);
	cpsw_ale_set_addr(ale_entry, addr);
	cpsw_ale_set_ucast_type(ale_entry, ALE_UCAST_PERSISTANT);
	cpsw_ale_set_secure(ale_entry, (flags & ALE_SECURE) ? 1 : 0);
	cpsw_ale_set_blocked(ale_entry, (flags & ALE_BLOCKED) ? 1 : 0);
	cpsw_ale_set_port_num(ale_entry, port);

	idx = cpsw_ale_match_addr(ale, addr);
	if (idx < 0)
		idx = cpsw_ale_match_free(ale);
	if (idx < 0)
		idx = cpsw_ale_find_ageable(ale);
	if (idx < 0)
		return -ENOMEM;

	cpsw_ale_write(ale, idx, ale_entry);
	return 0;
}

int cpsw_ale_del_ucast(struct cpsw_ale *ale, u8 *addr, int port)
{
	u32 ale_entry[ALE_ENTRY_WORDS] = {0, 0, 0};
	int idx;

	idx = cpsw_ale_match_addr(ale, addr);
	if (idx < 0)
		return -ENOENT;

	cpsw_ale_set_entry_type(ale_entry, ALE_TYPE_FREE);
	cpsw_ale_write(ale, idx, ale_entry);
	return 0;
}

int cpsw_ale_add_mcast(struct cpsw_ale *ale, u8 *addr, int port_mask)
{
	u32 ale_entry[ALE_ENTRY_WORDS] = {0, 0, 0};
	int idx, mask;

	idx = cpsw_ale_match_addr(ale, addr);
	if (idx >= 0)
		cpsw_ale_read(ale, idx, ale_entry);

	cpsw_ale_set_entry_type(ale_entry, ALE_TYPE_ADDR);
	cpsw_ale_set_addr(ale_entry, addr);
	cpsw_ale_set_mcast_state(ale_entry, ALE_MCAST_FWD_2);

	mask = cpsw_ale_get_port_mask(ale_entry);
	port_mask |= mask;
	cpsw_ale_set_port_mask(ale_entry, port_mask);

	if (idx < 0)
		idx = cpsw_ale_match_free(ale);
	if (idx < 0)
		idx = cpsw_ale_find_ageable(ale);
	if (idx < 0)
		return -ENOMEM;

	cpsw_ale_write(ale, idx, ale_entry);
	return 0;
}

int cpsw_ale_del_mcast(struct cpsw_ale *ale, u8 *addr, int port_mask)
{
	u32 ale_entry[ALE_ENTRY_WORDS] = {0, 0, 0};
	int idx, mask;

	idx = cpsw_ale_match_addr(ale, addr);
	if (idx < 0)
		return -ENOENT;

	cpsw_ale_read(ale, idx, ale_entry);
	mask = cpsw_ale_get_port_mask(ale_entry);
	port_mask = mask & ~port_mask;

	if (!port_mask)
		cpsw_ale_set_entry_type(ale_entry, ALE_TYPE_FREE);
	else
		cpsw_ale_set_port_mask(ale_entry, port_mask);

	cpsw_ale_write(ale, idx, ale_entry);
	return 0;
}

int cpsw_ale_add_vlan(struct cpsw_ale *ale, u16 vid, int port, int untag,
		      int reg_mcast, int unreg_mcast)
{
	u32 ale_entry[ALE_ENTRY_WORDS] = {0, 0, 0};
	int idx, mask;

	idx = cpsw_ale_match_vlan(ale, vid);
	if (idx >= 0)
		cpsw_ale_read(ale, idx, ale_entry);

	cpsw_ale_set_entry_type(ale_entry, ALE_TYPE_VLAN);
	cpsw_ale_set_vlan_id(ale_entry, vid);

	mask  = cpsw_ale_get_vlan_untag_force(ale_entry);
	if (untag)
		mask |= BIT(port);
	else
		mask &= ~BIT(port);
	cpsw_ale_set_vlan_untag_force(ale_entry, mask);

	mask  = cpsw_ale_get_vlan_reg_mcast(ale_entry);
	if (reg_mcast)
		mask |= BIT(port);
	else
		mask &= ~BIT(port);
	cpsw_ale_set_vlan_reg_mcast(ale_entry, mask);

	mask  = cpsw_ale_get_vlan_unreg_mcast(ale_entry);
	if (unreg_mcast)
		mask |= BIT(port);
	else
		mask &= ~BIT(port);
	cpsw_ale_set_vlan_unreg_mcast(ale_entry, mask);

	mask  = cpsw_ale_get_vlan_member_list(ale_entry);
	mask |= BIT(port);
	cpsw_ale_set_vlan_member_list(ale_entry, mask);

	if (idx < 0)
		idx = cpsw_ale_match_free(ale);
	if (idx < 0)
		idx = cpsw_ale_find_ageable(ale);
	if (idx < 0)
		return -ENOMEM;

	cpsw_ale_write(ale, idx, ale_entry);
	return 0;
}

int cpsw_ale_del_vlan(struct cpsw_ale *ale, u16 vid, int port)
{
	u32 ale_entry[ALE_ENTRY_WORDS] = {0, 0, 0};
	int idx, mask;

	idx = cpsw_ale_match_vlan(ale, vid);
	if (idx < 0)
		return -ENOENT;

	cpsw_ale_read(ale, idx, ale_entry);

	mask  = cpsw_ale_get_vlan_untag_force(ale_entry);
	mask &= ~BIT(port);
	cpsw_ale_set_vlan_untag_force(ale_entry, mask);

	mask  = cpsw_ale_get_vlan_reg_mcast(ale_entry);
	mask &= ~BIT(port);
	cpsw_ale_set_vlan_reg_mcast(ale_entry, mask);

	mask  = cpsw_ale_get_vlan_unreg_mcast(ale_entry);
	mask &= ~BIT(port);
	cpsw_ale_set_vlan_unreg_mcast(ale_entry, mask);

	mask  = cpsw_ale_get_vlan_member_list(ale_entry);
	mask &= ~BIT(port);
	if (!mask)
		cpsw_ale_set_entry_type(ale_entry, ALE_TYPE_FREE);
	else
		cpsw_ale_set_vlan_member_list(ale_entry, mask);

	cpsw_ale_write(ale, idx, ale_entry);
	return 0;
}

struct ale_control_info {
	int		offset, port_offset;
	int		shift, port_shift;
	int		bits;
};

#define CTRL_GLOBAL(bit)		{ALE_CONTROL, 0, bit, 0, 1}
#define CTRL_UNK(bit)			{ALE_UNKNOWNVLAN, 0, bit, 1, 1}
#define CTRL_PORTCTL(start, bits)	{ALE_PORTCTL, 4, start, 0, bits}

static struct ale_control_info ale_controls[] = {
	[ALE_ENABLE]			= CTRL_GLOBAL(31),
	[ALE_CLEAR]			= CTRL_GLOBAL(30),
	[ALE_AGEOUT]			= CTRL_GLOBAL(29),
	[ALE_VLAN_NOLEARN]		= CTRL_GLOBAL(7),
	[ALE_NO_PORT_VLAN]		= CTRL_GLOBAL(6),
	[ALE_OUI_DENY]			= CTRL_GLOBAL(5),
	[ALE_BYPASS]			= CTRL_GLOBAL(4),
	[ALE_RATE_LIMIT_TX]		= CTRL_GLOBAL(3),
	[ALE_VLAN_AWARE]		= CTRL_GLOBAL(2),
	[ALE_AUTH_ENABLE]		= CTRL_GLOBAL(1),
	[ALE_RATE_LIMIT]		= CTRL_GLOBAL(0),
	[ALE_PORT_STATE]		= CTRL_PORTCTL(0, 2),
	[ALE_PORT_DROP_UNTAGGED]	= CTRL_PORTCTL(2, 1),
	[ALE_PORT_DROP_UNK_VLAN]	= CTRL_PORTCTL(3, 1),
	[ALE_PORT_NOLEARN]		= CTRL_PORTCTL(4, 1),
	[ALE_PORT_MCAST_LIMIT]		= CTRL_PORTCTL(16, 8),
	[ALE_PORT_BCAST_LIMIT]		= CTRL_PORTCTL(24, 8),
	[ALE_PORT_UNK_VLAN_MEMBER]	= CTRL_UNK(0),
	[ALE_PORT_UNK_MCAST_FLOOD]	= CTRL_UNK(8),
	[ALE_PORT_UNTAGGED_EGRESS]	= CTRL_UNK(24),
	[ALE_PORT_UNK_REG_MCAST_FLOOD] = CTRL_UNK(16),
};

const char *ale_control_names[] = {
	[ALE_ENABLE]			= "ale_enable",
	[ALE_CLEAR]			= "ale_clear",
	[ALE_AGEOUT]			= "ale_ageout",
	[ALE_VLAN_NOLEARN]		= "ale_vlan_nolearn",
	[ALE_NO_PORT_VLAN]		= "ale_no_port_vlan",
	[ALE_OUI_DENY]			= "ale_oui_deny",
	[ALE_BYPASS]			= "ale_bypass",
	[ALE_RATE_LIMIT_TX]		= "ale_rate_limit_tx",
	[ALE_VLAN_AWARE]		= "ale_vlan_aware",
	[ALE_AUTH_ENABLE]		= "ale_auth_enable",
	[ALE_RATE_LIMIT]		= "ale_rate_limit",
	[ALE_PORT_STATE]		= "ale_port_state",
	[ALE_PORT_DROP_UNTAGGED]	= "ale_port_drop_untagged",
	[ALE_PORT_DROP_UNK_VLAN]	= "ale_port_drop_unk_vlan",
	[ALE_PORT_NOLEARN]		= "ale_port_nolearn",
	[ALE_PORT_MCAST_LIMIT]		= "ale_port_mcast_limit",
	[ALE_PORT_BCAST_LIMIT]		= "ale_port_bcast_limit",
	[ALE_PORT_UNK_VLAN_MEMBER]	= "ale_port_unk_vlan_member",
	[ALE_PORT_UNK_MCAST_FLOOD]	= "ale_port_unk_mcast_flood",
	[ALE_PORT_UNTAGGED_EGRESS]	= "ale_port_untagged_egress",
	[ALE_PORT_UNK_REG_MCAST_FLOOD]	= "ale_port_unk_reg_mcast_flood",
};

int cpsw_ale_control_set(struct cpsw_ale *ale, int port, int control,
			 int value)
{
	struct ale_control_info *info = &ale_controls[control];
	int offset, shift;
	u32 tmp, mask;

	if (control < 0 || control >= ARRAY_SIZE(ale_controls))
		return -EINVAL;

	if (info->port_offset == 0 && info->port_shift == 0)
		port = 0; /* global, port is a dont care */

	if (port < 0 || port > ale->ale_ports)
		return -EINVAL;

	mask = BITMASK(info->bits);
	if (value & ~mask)
		return -EINVAL;

	offset = info->offset + (port * info->port_offset);
	shift  = info->shift  + (port * info->port_shift);

	tmp = __raw_readl(ale->ale_regs + offset);
	tmp = (tmp & ~(mask << shift)) | (value << shift);
	__raw_writel(tmp, ale->ale_regs + offset);

	return 0;
}

int cpsw_ale_control_get(struct cpsw_ale *ale, int port, int control)
{
	struct ale_control_info *info = &ale_controls[control];
	int offset, shift;
	u32 tmp;

	if (control < 0 || control >= ARRAY_SIZE(ale_controls))
		return -EINVAL;

	if (info->port_offset == 0 && info->port_shift == 0)
		port = 0; /* global, port is a dont care */

	if (port < 0 || port > ale->ale_ports)
		return -EINVAL;

	offset = info->offset + (port * info->port_offset);
	shift  = info->shift  + (port * info->port_shift);

	tmp = __raw_readl(ale->ale_regs + offset) >> shift;
	return tmp & BITMASK(info->bits);
}

static ssize_t cpsw_ale_table_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	int len = SZ_4K, outlen = 0, idx;
	u32 ale_entry[ALE_ENTRY_WORDS];
	struct cpsw_ale *ale = table_attr_to_ale(attr);

	for (idx = 0; idx < ale->ale_entries; idx++) {
		cpsw_ale_read(ale, idx, ale_entry);
		outlen += cpsw_ale_dump_entry(idx, ale_entry, buf + outlen,
					      len - outlen);
	}
	return outlen;
}

static ssize_t cpsw_ale_table_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	u32 ale_entry[ALE_ENTRY_WORDS];
	int error, idx = -1;
	struct cpsw_ale *ale = table_attr_to_ale(attr);

	error = cpsw_ale_parse_entry(&idx, ale_entry, buf, count);
	if (error < 0)
		return error;

	if (idx < 0)
		idx = cpsw_ale_match_free(ale);
	if (idx < 0)
		idx = cpsw_ale_find_ageable(ale);
	if (idx < 0)
		return -ENOMEM;

	cpsw_ale_write(ale, idx, ale_entry);

	return count;
}

DEVICE_ATTR(ale_table, S_IRUGO | S_IWUSR, cpsw_ale_table_show,
	    cpsw_ale_table_store);

static void cpsw_ale_timer(unsigned long arg)
{
	struct cpsw_ale *ale = (struct cpsw_ale *)arg;

	cpsw_ale_control_set(ale, 0, ALE_AGEOUT, 1);

	if (ale->ageout) {
		ale->timer.expires = jiffies + ale->ageout;
		add_timer(&ale->timer);
	}
}

int cpsw_ale_set_ageout(struct cpsw_ale *ale, int ageout)
{
	del_timer_sync(&ale->timer);
	ale->ageout = ageout * HZ;
	if (ale->ageout) {
		ale->timer.expires = jiffies + ale->ageout;
		add_timer(&ale->timer);
	}
	return 0;
}

void cpsw_ale_start(struct cpsw_ale *ale)
{
	u32 rev;
	int ret;

	rev = __raw_readl(ale->ale_regs + ALE_IDVER);
	dev_dbg(ale->params.dev, "initialized cpsw ale revision %d.%d\n",
		(rev >> 8) & 0xff, rev & 0xff);
	cpsw_ale_control_set(ale, 0, ALE_ENABLE, 1);
	cpsw_ale_control_set(ale, 0, ALE_CLEAR, 1);

	ale->ale_table_attr = dev_attr_ale_table;
	ret = device_create_file(ale->params.dev, &ale->ale_table_attr);
	WARN_ON(ret < 0);

	init_timer(&ale->timer);
	ale->timer.data	    = (unsigned long)ale;
	ale->timer.function = cpsw_ale_timer;
	if (ale->ageout) {
		ale->timer.expires = jiffies + ale->ageout;
		add_timer(&ale->timer);
	}
}

void cpsw_ale_stop(struct cpsw_ale *ale)
{
	del_timer_sync(&ale->timer);
	device_remove_file(ale->params.dev, &ale->ale_table_attr);
}

struct cpsw_ale *cpsw_ale_create(struct cpsw_ale_params *params)
{
	struct cpsw_ale *ale;
	int ret;

	ret = -ENOMEM;
	ale = kzalloc(sizeof(*ale), GFP_KERNEL);
	if (WARN_ON(!ale))
		return NULL;

	ale->params = *params;
	ale->ageout = ale->params.ale_ageout * HZ;

	return ale;
}

int cpsw_ale_destroy(struct cpsw_ale *ale)
{
	if (!ale)
		return -EINVAL;
	cpsw_ale_stop(ale);
	cpsw_ale_control_set(ale, 0, ALE_ENABLE, 0);
	kfree(ale);
	return 0;
}
