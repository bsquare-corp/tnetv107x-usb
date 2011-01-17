/*
 * Texas Instruments Ethernet Switch Driver
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
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/timer.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/if_ether.h>
#include <linux/etherdevice.h>
#include <linux/netdevice.h>
#include <linux/phy.h>

#include <mach/cpsw.h>

#include "cpsw_ale.h"
#include "davinci_cpdma.h"

#define CPSW_DEBUG	(NETIF_MSG_HW		| NETIF_MSG_WOL		| \
			 NETIF_MSG_DRV		| NETIF_MSG_LINK	| \
			 NETIF_MSG_IFUP		| NETIF_MSG_INTR	| \
			 NETIF_MSG_PROBE	| NETIF_MSG_TIMER	| \
			 NETIF_MSG_IFDOWN	| NETIF_MSG_RX_ERR	| \
			 NETIF_MSG_TX_ERR	| NETIF_MSG_TX_DONE	| \
			 NETIF_MSG_PKTDATA	| NETIF_MSG_TX_QUEUED	| \
			 NETIF_MSG_RX_STATUS)

#define msg(level, type, format, ...)				\
do {								\
	if (netif_msg_##type(priv) && net_ratelimit())		\
		dev_##level(priv->dev, format, ## __VA_ARGS__);	\
} while (0)

#define CPSW_POLL_WEIGHT	64
#define CPSW_MIN_PACKET_SIZE	60
#define CPSW_MAX_PACKET_SIZE	(1500 + 14 + 4 + 4)

static int debug_level;
module_param(debug_level, int, 0);
MODULE_PARM_DESC(debug_level, "cpsw debug level (NETIF_MSG bits)");

static int ale_ageout = 10;
module_param(ale_ageout, int, 0);
MODULE_PARM_DESC(ale_ageout, "cpsw ale ageout interval (seconds)");

static int rx_packet_max = CPSW_MAX_PACKET_SIZE;
module_param(rx_packet_max, int, 0);
MODULE_PARM_DESC(rx_packet_max, "maximum receive packet size (bytes)");

struct cpsw_regs {
	u32	id_ver;
	u32	control;
#define CPSW_CONTROL_VLAN_AWARE		BIT(9)
	u32	soft_reset;
	u32	stat_port_en;
	u32	ptype;
};

struct cpsw_slave_regs {
	u32	max_blks;
	u32	blk_cnt;
	u32	flow_thresh;
	u32	port_vlan;
	u32	tx_pri_map;
	u32	gap_thresh;
	u32	sa_lo;
	u32	sa_hi;
};

struct cpsw_host_regs {
	u32	max_blks;
	u32	blk_cnt;
	u32	flow_thresh;
	u32	port_vlan;
	u32	tx_pri_map;
	u32	cpdma_tx_pri_map;
	u32	cpdma_rx_chan_map;
};

struct cpsw_sliver_regs {
	u32	id_ver;
	u32	mac_control;
	u32	mac_status;
	u32	soft_reset;
	u32	rx_maxlen;
	u32	__reserved_0;
	u32	rx_pause;
	u32	tx_pause;
	u32	__reserved_1;
	u32	rx_pri_map;
};

struct cpsw_hw_stats {
	u32	rxgoodframes;
	u32	rxbroadcastframes;
	u32	rxmulticastframes;
	u32	rxpauseframes;
	u32	rxcrcerrors;
	u32	rxaligncodeerrors;
	u32	rxoversizedframes;
	u32	rxjabberframes;
	u32	rxundersizedframes;
	u32	rxfragments;
	u32	__pad_0[2];
	u32	rxoctets;
	u32	txgoodframes;
	u32	txbroadcastframes;
	u32	txmulticastframes;
	u32	txpauseframes;
	u32	txdeferredframes;
	u32	txcollisionframes;
	u32	txsinglecollframes;
	u32	txmultcollframes;
	u32	txexcessivecollisions;
	u32	txlatecollisions;
	u32	txunderrun;
	u32	txcarriersenseerrors;
	u32	txoctets;
	u32	octetframes64;
	u32	octetframes65t127;
	u32	octetframes128t255;
	u32	octetframes256t511;
	u32	octetframes512t1023;
	u32	octetframes1024tup;
	u32	netoctets;
	u32	rxsofoverruns;
	u32	rxmofoverruns;
	u32	rxdmaoverruns;
};

struct cpsw_slave {
	struct cpsw_slave_regs __iomem	*regs;
	struct cpsw_sliver_regs __iomem	*sliver;
	int				slave_num;
	u32				mac_control;
	struct cpsw_slave_data		*data;
	struct phy_device		*phy;
};

struct cpsw_priv {
	spinlock_t			lock;
	struct platform_device		*pdev;
	struct net_device		*ndev;
	struct resource			*res;
	struct napi_struct		napi;
#define napi_to_priv(napi)	container_of(napi, struct cpsw_priv, napi)
	struct device			*dev;
	struct cpsw_platform_data	data;
	struct cpsw_regs __iomem	*regs;
	struct cpsw_hw_stats __iomem	*hw_stats;
	struct cpsw_host_regs __iomem	*host_port_regs;
	u32				msg_enable;
	struct net_device_stats		stats;
	int				rx_packet_max;
	int				host_port;
	struct clk			*clk;
	struct cpsw_slave		*slaves;
#define for_each_slave(priv, func, arg...)			\
	do {							\
		int idx;					\
		for (idx = 0; idx < (priv)->data.slaves; idx++)	\
			(func)((priv)->slaves + idx, ##arg);	\
	} while (0)

	struct cpdma_ctlr		*dma;
	struct cpdma_chan		*txch, *rxch;
	struct cpsw_ale			*ale;
};

void cpsw_tx_handler(void *token, int len, int status)
{
	struct sk_buff		*skb = token;
	struct net_device	*ndev = skb->dev;
	struct cpsw_priv	*priv = netdev_priv(ndev);

	if (unlikely(netif_queue_stopped(ndev)))
		netif_start_queue(ndev);
	priv->stats.tx_packets++;
	priv->stats.tx_bytes += len;
	dev_kfree_skb_any(skb);
}

void cpsw_rx_handler(void *token, int len, int status)
{
	struct sk_buff		*skb = token;
	struct net_device	*ndev = skb->dev;
	struct cpsw_priv	*priv = netdev_priv(ndev);
	int			ret;

	if (likely(status >= 0)) {
		skb_put(skb, len);
		skb->protocol = eth_type_trans(skb, ndev);
		netif_receive_skb(skb);
		priv->stats.rx_bytes += len;
		priv->stats.rx_packets++;
		skb = NULL;
	}


	if (unlikely(!netif_running(ndev))) {
		if (skb)
			dev_kfree_skb_any(skb);
		return;
	}

	if (likely(!skb)) {
		skb = netdev_alloc_skb_ip_align(ndev, priv->rx_packet_max);
		if (WARN_ON(!skb))
			return;
	}

	ret = cpdma_chan_submit(priv->rxch, skb, skb->data,
				skb_tailroom(skb), GFP_KERNEL);
	WARN_ON(ret < 0);
}

static irqreturn_t cpsw_interrupt(int irq, void *dev_id)
{
	struct cpsw_priv *priv = dev_id;

	if (likely(netif_running(priv->ndev)))
		napi_schedule(&priv->napi);
	return IRQ_HANDLED;
}

static int cpsw_poll(struct napi_struct *napi, int budget)
{
	struct cpsw_priv	*priv = napi_to_priv(napi);
	int			num_tx, num_rx;

	num_tx = cpdma_chan_process(priv->txch, 128);
	num_rx = cpdma_chan_process(priv->rxch, budget);

	if (num_rx || num_tx)
		msg(dbg, intr, "poll %d rx, %d tx pkts\n", num_rx, num_tx);

	if (num_rx < budget) {
		napi_complete(napi);
		cpdma_ctlr_eoi(priv->dma);
	}

	return num_rx;
}

static inline void soft_reset(const char *module, void __iomem *reg)
{
	unsigned long timeout = jiffies + HZ;

	__raw_writel(1, reg);
	do {
		cpu_relax();
	} while ((__raw_readl(reg) & 1) && time_after(timeout, jiffies));

	WARN(__raw_readl(reg) & 1, "failed to soft-reset %s\n", module);
}

#define mac_hi(mac)	(((mac)[0] << 0) | ((mac)[1] << 8) |	\
			 ((mac)[2] << 16) | ((mac)[3] << 24))
#define mac_lo(mac)	(((mac)[4] << 0) | ((mac)[5] << 8))

static void cpsw_set_slave_mac(struct cpsw_slave *slave,
			       struct cpsw_priv *priv)
{
	__raw_writel(mac_hi(priv->ndev->dev_addr), &slave->regs->sa_hi);
	__raw_writel(mac_lo(priv->ndev->dev_addr), &slave->regs->sa_lo);
}

static void _cpsw_adjust_link(struct cpsw_slave *slave,
			      struct cpsw_priv *priv, bool *link)
{
	struct phy_device	*phy = slave->phy;
	u32			mac_control = 0;

	if (phy->link) {
		mac_control = priv->data.mac_control;
		if (phy->speed == 1000)
			mac_control |= BIT(7);	/* GIGABITEN	*/
		if (phy->duplex)
			mac_control |= BIT(0);	/* FULLDUPLEXEN	*/
		*link = true;
	} else {
		mac_control = 0;
	}

	if (mac_control != slave->mac_control) {
		phy_print_status(phy);
		__raw_writel(mac_control, &slave->sliver->mac_control);
	}

	slave->mac_control = mac_control;
}

static void cpsw_adjust_link(struct net_device *ndev)
{
	struct cpsw_priv	*priv = netdev_priv(ndev);
	bool			link = false;

	for_each_slave(priv, _cpsw_adjust_link, priv, &link);

	if (link) {
		netif_carrier_on(ndev);
		if (netif_running(ndev))
			netif_wake_queue(ndev);
	} else {
		netif_carrier_off(ndev);
		netif_stop_queue(ndev);
	}
}

static inline int __show_stat(char *buf, int maxlen, const char* name, u32 val)
{
	static char *leader = "........................................";

	if (!val)
		return 0;
	else
		return snprintf(buf, maxlen, "%s %s %10d\n", name,
				leader + strlen(name), val);
}

static ssize_t cpsw_hw_stats_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct net_device	*ndev = to_net_dev(dev);
	struct cpsw_priv	*priv = netdev_priv(ndev);
	int			len = 0;
	struct cpdma_chan_stats	dma_stats;

#define show_stat(x) do {						\
	len += __show_stat(buf + len, SZ_4K - len, #x,			\
			   __raw_readl(&priv->hw_stats->x));		\
} while (0)

#define show_dma_stat(x) do {						\
	len += __show_stat(buf + len, SZ_4K - len, #x, dma_stats.x);	\
} while (0)

	len += snprintf(buf + len, SZ_4K - len, "CPSW Statistics:\n");
	show_stat(rxgoodframes);	show_stat(rxbroadcastframes);
	show_stat(rxmulticastframes);	show_stat(rxpauseframes);
	show_stat(rxcrcerrors);		show_stat(rxaligncodeerrors);
	show_stat(rxoversizedframes);	show_stat(rxjabberframes);
	show_stat(rxundersizedframes);	show_stat(rxfragments);
	show_stat(rxoctets);		show_stat(txgoodframes);
	show_stat(txbroadcastframes);	show_stat(txmulticastframes);
	show_stat(txpauseframes);	show_stat(txdeferredframes);
	show_stat(txcollisionframes);	show_stat(txsinglecollframes);
	show_stat(txmultcollframes);	show_stat(txexcessivecollisions);
	show_stat(txlatecollisions);	show_stat(txunderrun);
	show_stat(txcarriersenseerrors); show_stat(txoctets);
	show_stat(octetframes64);	show_stat(octetframes65t127);
	show_stat(octetframes128t255);	show_stat(octetframes256t511);
	show_stat(octetframes512t1023);	show_stat(octetframes1024tup);
	show_stat(netoctets);		show_stat(rxsofoverruns);
	show_stat(rxmofoverruns);	show_stat(rxdmaoverruns);

	cpdma_chan_get_stats(priv->rxch, &dma_stats);
	len += snprintf(buf + len, SZ_4K - len, "\nRX DMA Statistics:\n");
	show_dma_stat(head_enqueue);	show_dma_stat(tail_enqueue);
	show_dma_stat(pad_enqueue);	show_dma_stat(misqueued);
	show_dma_stat(desc_alloc_fail);	show_dma_stat(pad_alloc_fail);
	show_dma_stat(runt_receive_buff); show_dma_stat(runt_transmit_buff);
	show_dma_stat(empty_dequeue);	show_dma_stat(busy_dequeue);
	show_dma_stat(good_dequeue);	show_dma_stat(teardown_dequeue);
	show_dma_stat(requeue);

	cpdma_chan_get_stats(priv->txch, &dma_stats);
	len += snprintf(buf + len, SZ_4K - len, "\nTX DMA Statistics:\n");
	show_dma_stat(head_enqueue);	show_dma_stat(tail_enqueue);
	show_dma_stat(pad_enqueue);	show_dma_stat(misqueued);
	show_dma_stat(desc_alloc_fail);	show_dma_stat(pad_alloc_fail);
	show_dma_stat(runt_receive_buff); show_dma_stat(runt_transmit_buff);
	show_dma_stat(empty_dequeue);	show_dma_stat(busy_dequeue);
	show_dma_stat(good_dequeue);	show_dma_stat(teardown_dequeue);
	show_dma_stat(requeue);

	return len;
}

DEVICE_ATTR(hw_stats, S_IRUGO, cpsw_hw_stats_show, NULL);

static ssize_t cpsw_hw_control_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct net_device *ndev = container_of(dev, struct net_device, dev);
	struct cpsw_priv *priv = netdev_priv(ndev);
	int port, len = 0, control, value;

	for (port = 0; port < priv->data.slaves; port++) {
		struct cpsw_slave *slave = priv->slaves + port;
		value = __raw_readl(&slave->regs->port_vlan);
		len += snprintf(buf + len, SZ_4K - len, "port_vlan.%d=%d\n",
				slave->slave_num, value);
	}

	value = __raw_readl(&priv->host_port_regs->port_vlan);
	len += snprintf(buf + len, SZ_4K - len, "port_vlan.%d=%d\n",
			priv->host_port, value);

	for (control = 0; control < __ALE_MAX_GLOBAL_CONTROL; control++) {
		value = cpsw_ale_control_get(priv->ale, 0, control);
		len += snprintf(buf + len, SZ_4K - len, "%s=%d\n",
				ale_control_names[control], value);
	}
	for (; control < __ALE_MAX_CONTROL; control++) {
		for (port = 0; port <= priv->data.slaves; port++) {
			value = cpsw_ale_control_get(priv->ale, port, control);
			len += snprintf(buf + len, SZ_4K - len, "%s.%d=%d\n",
					ale_control_names[control], port, value);
		}
	}
	return len;
}

static ssize_t cpsw_hw_control_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct net_device *ndev = container_of(dev, struct net_device, dev);
	struct cpsw_priv *priv = netdev_priv(ndev);
	char ctrl_str[33], *end;
	int port = 0, value, len, ret, control;

	len = strcspn(buf, ".=");
	if (len >= 32)
		return -ENOMEM;
	strncpy(ctrl_str, buf, len);
	ctrl_str[len] = '\0';
	buf += len;

	if (*buf == '.') {
		port = simple_strtoul(buf + 1, &end, 0);
		buf = end;
	}

	if (*buf != '=')
		return -EINVAL;

	value = simple_strtoul(buf + 1, NULL, 0);

	if (strcmp(ctrl_str, "port_vlan") == 0) {
		if (port == priv->host_port) {
			__raw_writel(value, &priv->host_port_regs->port_vlan);
		} else {
			struct cpsw_slave *slave = priv->slaves + port;
			__raw_writel(value, &slave->regs->port_vlan);
		}
		return count;
	}

	for (control = 0; control < __ALE_MAX_CONTROL; control++)
		if (strcmp(ctrl_str, ale_control_names[control]) == 0)
			break;

	if (control >= __ALE_MAX_CONTROL)
		return -ENOENT;

	ret = cpsw_ale_control_set(priv->ale, port, control, value);
	if (ret < 0)
		return ret;
	return count;
}

DEVICE_ATTR(hw_control, S_IRUGO | S_IWUSR, cpsw_hw_control_show,
	    cpsw_hw_control_store);

static void cpsw_slave_open(struct cpsw_slave *slave, struct cpsw_priv *priv)
{
	char name[32];

	sprintf(name, "slave-%d", slave->slave_num);

	soft_reset(name, &slave->sliver->soft_reset);

	/* setup priority mapping */
	__raw_writel(0x76543210, &slave->sliver->rx_pri_map);
	__raw_writel(0x33221100, &slave->regs->tx_pri_map);

	/* setup max packet size, and mac address */
	__raw_writel(priv->rx_packet_max, &slave->sliver->rx_maxlen);
	cpsw_set_slave_mac(slave, priv);

	slave->mac_control = 0;	/* no link yet */

	cpsw_ale_add_vlan(priv->ale, 0, slave->slave_num, 1, 1, 1);

	cpsw_ale_control_set(priv->ale, slave->slave_num,
			     ALE_PORT_STATE, ALE_PORT_STATE_FORWARD);

	cpsw_ale_add_mcast(priv->ale, priv->ndev->broadcast,
			   1 << slave->slave_num);

	slave->phy = phy_connect(priv->ndev, slave->data->phy_id,
				 &cpsw_adjust_link, 0, slave->data->phy_if);
	if (IS_ERR(slave->phy)) {
		msg(err, ifup, "phy %s not found on slave %d\n",
		    slave->data->phy_id, slave->slave_num);
		slave->phy = NULL;
	} else {
		phy_start(slave->phy);
	}
}

static void cpsw_init_host_port(struct cpsw_priv *priv)
{
	/* soft reset the controller and initialize ale */
	soft_reset("cpsw", &priv->regs->soft_reset);
	cpsw_ale_start(priv->ale);

	/* switch to vlan aware mode */
	__raw_writel(CPSW_CONTROL_VLAN_AWARE, &priv->regs->control);
	cpsw_ale_control_set(priv->ale, 0, ALE_VLAN_AWARE, 1);

	/* setup host port priority mapping */
	__raw_writel(0x76543210, &priv->host_port_regs->cpdma_tx_pri_map);
	__raw_writel(0, &priv->host_port_regs->cpdma_rx_chan_map);

	cpsw_ale_add_vlan(priv->ale, 0, priv->host_port, 0, 1, 1);

	cpsw_ale_control_set(priv->ale, priv->host_port,
			     ALE_PORT_UNK_VLAN_MEMBER, 1);
	cpsw_ale_control_set(priv->ale, priv->host_port,
			     ALE_PORT_UNK_MCAST_FLOOD, 1);
	cpsw_ale_control_set(priv->ale, priv->host_port,
			     ALE_PORT_UNK_REG_MCAST_FLOOD, 1);
	cpsw_ale_control_set(priv->ale, priv->host_port,
			     ALE_PORT_STATE, ALE_PORT_STATE_FORWARD);

	cpsw_ale_add_ucast(priv->ale, priv->ndev->dev_addr, priv->host_port,
			   ALE_SECURE);
	cpsw_ale_add_mcast(priv->ale, priv->ndev->broadcast,
			   1 << priv->host_port);
}

static int cpsw_ndo_open(struct net_device *ndev)
{
	struct cpsw_priv *priv = netdev_priv(ndev);
	int i, ret;
	u32 reg;

	netif_carrier_off(ndev);

	ret = clk_enable(priv->clk);
	if (ret < 0) {
		dev_err(priv->dev, "unable to turn on device clock\n");
		return ret;
	}

	ret = device_create_file(&ndev->dev, &dev_attr_hw_stats);
	if (ret < 0) {
		dev_err(priv->dev, "unable to add device attr\n");
		return ret;
	}

	ret = device_create_file(&ndev->dev, &dev_attr_hw_control);
	if (ret < 0) {
		dev_err(priv->dev, "unable to add device attr\n");
		return ret;
	}

	if (priv->data.phy_control)
		(*priv->data.phy_control)(true);

	reg = __raw_readl(&priv->regs->id_ver);

	msg(info, ifup, "initializing cpsw version %d.%d (%d)\n",
	    (reg >> 8 & 0x7), reg & 0xff, (reg >> 11) & 0x1f);

	/* initialize host and slave ports */
	cpsw_init_host_port(priv);
	for_each_slave(priv, cpsw_slave_open, priv);

	/* setup tx dma to fixed prio and zero offset */
	cpdma_control_set(priv->dma, CPDMA_TX_PRIO_FIXED, 1);
	cpdma_control_set(priv->dma, CPDMA_RX_BUFFER_OFFSET, 0);

	/* disable priority elevation and enable statistics on all ports */
	__raw_writel(0, &priv->regs->ptype);

	/* enable statistics collection only on the host port */
	__raw_writel(BIT(priv->host_port), &priv->regs->stat_port_en);

	if (WARN_ON(!priv->data.rx_descs))
		priv->data.rx_descs = 128;

	for (i = 0; i < priv->data.rx_descs; i++) {
		struct sk_buff *skb;

		ret = -ENOMEM;
		skb = netdev_alloc_skb_ip_align(priv->ndev,
						priv->rx_packet_max);
		if (!skb)
			break;
		ret = cpdma_chan_submit(priv->rxch, skb, skb->data,
					skb_tailroom(skb), GFP_KERNEL);
		if (WARN_ON(ret < 0))
			break;
	}
	/* continue even if we didn't manage to submit all receive descs */
	msg(info, ifup, "submitted %d rx descriptors\n", i);

	cpdma_ctlr_start(priv->dma);
	cpdma_ctlr_int_ctrl(priv->dma, true);
	napi_enable(&priv->napi);

	return 0;
}

static void cpsw_slave_stop(struct cpsw_slave *slave, struct cpsw_priv *priv)
{
	if (!slave->phy)
		return;
	phy_stop(slave->phy);
	phy_disconnect(slave->phy);
	slave->phy = NULL;
}

static int cpsw_ndo_stop(struct net_device *ndev)
{
	struct cpsw_priv *priv = netdev_priv(ndev);

	msg(info, ifdown, "shutting down cpsw device\n");
	cpdma_ctlr_int_ctrl(priv->dma, false);
	cpdma_ctlr_stop(priv->dma);
	netif_stop_queue(priv->ndev);
	napi_disable(&priv->napi);
	netif_carrier_off(priv->ndev);
	cpsw_ale_stop(priv->ale);
	device_remove_file(&ndev->dev, &dev_attr_hw_stats);
	device_remove_file(&ndev->dev, &dev_attr_hw_control);
	for_each_slave(priv, cpsw_slave_stop, priv);
	if (priv->data.phy_control)
		(*priv->data.phy_control)(false);
	clk_disable(priv->clk);
	return 0;
}

static netdev_tx_t cpsw_ndo_start_xmit(struct sk_buff *skb,
				       struct net_device *ndev)
{
	struct cpsw_priv *priv = netdev_priv(ndev);
	int ret;

	ndev->trans_start = jiffies;

	ret = skb_padto(skb, CPSW_MIN_PACKET_SIZE);
	if (unlikely(ret < 0)) {
		msg(err, tx_err, "packet pad failed");
		goto fail;
	}

	ret = cpdma_chan_submit(priv->txch, skb, skb->data,
				skb->len, GFP_KERNEL);
	if (unlikely(ret != 0)) {
		msg(err, tx_err, "desc submit failed");
		goto fail;
	}

	return NETDEV_TX_OK;
fail:
	priv->stats.tx_dropped++;
	netif_stop_queue(ndev);
	return NETDEV_TX_BUSY;
}

static void cpsw_ndo_change_rx_flags(struct net_device *ndev, int flags)
{
	/*
	 * The switch cannot operate in promiscuous mode without substantial
	 * headache.  For promiscuous mode to work, we would need to put the
	 * ALE in bypass mode and route all traffic to the host port.
	 * Subsequently, the host will need to operate as a "bridge", learn,
	 * and flood as needed.  For now, we simply complain here and
	 * do nothing about it :-)
	 */
	if ((flags & IFF_PROMISC) && (ndev->flags & IFF_PROMISC))
		dev_err(&ndev->dev, "promiscuity ignored!\n");

	/*
	 * The switch cannot filter multicast traffic unless it is configured
	 * in "VLAN Aware" mode.  Unfortunately, VLAN awareness requires a
	 * whole bunch of additional logic that this driver does not implement
	 * at present.
	 */
	if ((flags & IFF_ALLMULTI) && !(ndev->flags & IFF_ALLMULTI))
		dev_err(&ndev->dev, "multicast traffic cannot be filtered!\n");
}

static void cpsw_ndo_tx_timeout(struct net_device *ndev)
{
	struct cpsw_priv *priv = netdev_priv(ndev);

	msg(err, tx_err, "transmit timeout, restarting dma");
	priv->stats.tx_errors++;
	cpdma_ctlr_int_ctrl(priv->dma, false);
	cpdma_chan_stop(priv->txch);
	cpdma_chan_start(priv->txch);
	cpdma_ctlr_int_ctrl(priv->dma, true);
}

static struct net_device_stats *cpsw_ndo_get_stats(struct net_device *ndev)
{
	struct cpsw_priv *priv = netdev_priv(ndev);
	return &priv->stats;
}

static void cpsw_ndo_poll_controller(struct net_device *ndev)
{
	struct cpsw_priv *priv = netdev_priv(ndev);

	cpdma_ctlr_int_ctrl(priv->dma, false);
	cpsw_interrupt(ndev->irq, priv);
	cpdma_ctlr_int_ctrl(priv->dma, true);
}

static const struct net_device_ops cpsw_netdev_ops = {
	.ndo_open		= cpsw_ndo_open,
	.ndo_stop		= cpsw_ndo_stop,
	.ndo_start_xmit		= cpsw_ndo_start_xmit,
	.ndo_change_rx_flags	= cpsw_ndo_change_rx_flags,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_tx_timeout		= cpsw_ndo_tx_timeout,
	.ndo_get_stats		= cpsw_ndo_get_stats,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller	= cpsw_ndo_poll_controller,
#endif
};

static void cpsw_get_drvinfo(struct net_device *ndev,
			     struct ethtool_drvinfo *info)
{
	struct cpsw_priv *priv = netdev_priv(ndev);
	strcpy(info->driver, "TI CPSW Driver v1.0");
	strcpy(info->version, "1.0");
	strcpy(info->bus_info, priv->pdev->name);
}

static u32 cpsw_get_msglevel(struct net_device *ndev)
{
	struct cpsw_priv *priv = netdev_priv(ndev);
	return priv->msg_enable;
}

static void cpsw_set_msglevel(struct net_device *ndev, u32 value)
{
	struct cpsw_priv *priv = netdev_priv(ndev);
	priv->msg_enable = value;
}

static const struct ethtool_ops cpsw_ethtool_ops = {
	.get_drvinfo	= cpsw_get_drvinfo,
	.get_msglevel	= cpsw_get_msglevel,
	.set_msglevel	= cpsw_set_msglevel,
	.get_link	= ethtool_op_get_link,
};

static void cpsw_slave_init(struct cpsw_slave *slave, struct cpsw_priv *priv)
{
	void __iomem		*regs = priv->regs;
	int			slave_num = slave->slave_num;
	struct cpsw_slave_data	*data = priv->data.slave_data + slave_num;

	slave->data	= data;
	slave->regs	= regs + data->slave_reg_ofs;
	slave->sliver	= regs + data->sliver_reg_ofs;
}

static int __devinit cpsw_probe(struct platform_device *pdev)
{
	struct cpsw_platform_data	*data = pdev->dev.platform_data;
	struct net_device		*ndev;
	struct cpsw_priv		*priv;
	struct cpdma_params		dma_params;
	struct cpsw_ale_params		ale_params;
	void __iomem			*regs;
	int ret = 0, i;

	if (!data) {
		pr_err("cpsw: platform data missing\n");
		return -ENODEV;
	}

	ndev = alloc_etherdev(sizeof(struct cpsw_priv));
	if (!ndev) {
		pr_err("cpsw: error allocating net_device\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, ndev);
	priv = netdev_priv(ndev);
	spin_lock_init(&priv->lock);
	priv->data = *data;
	priv->pdev = pdev;
	priv->ndev = ndev;
	priv->dev  = &ndev->dev;
	priv->msg_enable = netif_msg_init(debug_level, CPSW_DEBUG);
	priv->rx_packet_max = max(rx_packet_max, 128);

	if (is_valid_ether_addr(data->mac_addr))
		memcpy(ndev->dev_addr, data->mac_addr, ETH_ALEN);
	else
		random_ether_addr(ndev->dev_addr);

	priv->slaves = kzalloc(sizeof(struct cpsw_slave) * data->slaves,
			       GFP_KERNEL);
	if (!priv->slaves) {
		dev_err(priv->dev, "failed to allocate slave ports\n");
		ret = -EBUSY;
		goto clean_ndev_ret;
	}
	for (i = 0; i < data->slaves; i++)
		priv->slaves[i].slave_num = i;

	priv->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(priv->clk)) {
		dev_err(priv->dev, "failed to get device clock\n");
		ret = -EBUSY;
		goto clean_slaves_ret;
	}

	priv->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!priv->res) {
		dev_err(priv->dev, "error getting i/o resource\n");
		ret = -ENOENT;
		goto clean_clk_ret;
	}

	if (!request_mem_region(priv->res->start, resource_size(priv->res),
				ndev->name)) {
		dev_err(priv->dev, "failed request i/o region\n");
		ret = -ENXIO;
		goto clean_clk_ret;
	}

	regs = ioremap(priv->res->start, resource_size(priv->res));
	if (!regs) {
		dev_err(priv->dev, "unable to map i/o region\n");
		goto clean_iores_ret;
	}
	priv->regs = regs;
	priv->host_port = data->slaves;
	priv->host_port_regs = regs + data->host_port_reg_ofs;
	priv->hw_stats = regs + data->hw_stats_reg_ofs;

	for_each_slave(priv, cpsw_slave_init, priv);

	memset(&dma_params, 0, sizeof(dma_params));
	dma_params.dev			= &pdev->dev;
	dma_params.dmaregs		= regs + data->cpdma_reg_ofs;
	dma_params.rxthresh		= regs + data->cpdma_reg_ofs + 0x0c0;
	dma_params.rxfree		= regs + data->cpdma_reg_ofs + 0x0e0;
	dma_params.txhdp		= regs + data->cpdma_reg_ofs + 0x100;
	dma_params.rxhdp		= regs + data->cpdma_reg_ofs + 0x120;
	dma_params.txcp			= regs + data->cpdma_reg_ofs + 0x140;
	dma_params.rxcp			= regs + data->cpdma_reg_ofs + 0x160;
	dma_params.num_chan		= data->channels;
	dma_params.has_soft_reset	= true;
	dma_params.min_packet_size	= CPSW_MIN_PACKET_SIZE;
	dma_params.desc_mem_size	= SZ_64K;
	dma_params.desc_align		= 16;
	dma_params.has_ext_regs		= true;

	priv->dma = cpdma_ctlr_create(&dma_params);
	if (!priv->dma) {
		dev_err(priv->dev, "error initializing dma\n");
		ret = -ENOMEM;
		goto clean_iomap_ret;
	}

	priv->txch = cpdma_chan_create(priv->dma, tx_chan_num(0),
				       cpsw_tx_handler);
	priv->rxch = cpdma_chan_create(priv->dma, rx_chan_num(0),
				       cpsw_rx_handler);

	if (WARN_ON(!priv->txch || !priv->rxch)) {
		dev_err(priv->dev, "error initializing dma channels\n");
		ret = -ENOMEM;
		goto clean_dma_ret;
	}

	memset(&ale_params, 0, sizeof(ale_params));
	ale_params.dev			= &ndev->dev;
	ale_params.ale_regs		= regs + data->ale_reg_ofs;
	ale_params.ale_ageout		= ale_ageout;
	ale_params.ale_entries		= data->ale_entries;
	ale_params.ale_ports		= data->slaves;

	priv->ale = cpsw_ale_create(&ale_params);
	if (!priv->ale) {
		dev_err(priv->dev, "error initializing ale engine\n");
		ret = -ENODEV;
		goto clean_dma_ret;
	}

	ndev->irq = platform_get_irq(pdev, 0);
	if (ndev->irq < 0) {
		dev_err(priv->dev, "error getting irq resource\n");
		ret = -ENOENT;
		goto clean_ale_ret;
	}

	ret = request_irq(ndev->irq, cpsw_interrupt, 0, dev_name(&pdev->dev),
			  priv);
	if (ret < 0) {
		dev_err(priv->dev, "error attaching irq handler\n");
		goto clean_ale_ret;
	}

	ndev->flags |= IFF_ALLMULTI;	/* see cpsw_ndo_change_rx_flags() */

	ndev->netdev_ops = &cpsw_netdev_ops;
	SET_ETHTOOL_OPS(ndev, &cpsw_ethtool_ops);
	netif_napi_add(ndev, &priv->napi, cpsw_poll, CPSW_POLL_WEIGHT);

	/* register the network device */
	SET_NETDEV_DEV(ndev, &pdev->dev);
	ret = register_netdev(ndev);
	if (ret) {
		dev_err(priv->dev, "error registering net device\n");
		ret = -ENODEV;
		goto clean_irq_ret;
	}

	pr_info("registered %s device, (regs %x, irq %d)\n", ndev->name,
		priv->res->start, ndev->irq);

	return 0;

clean_irq_ret:
	free_irq(ndev->irq, priv);
clean_ale_ret:
	cpsw_ale_destroy(priv->ale);
clean_dma_ret:
	cpdma_chan_destroy(priv->txch);
	cpdma_chan_destroy(priv->rxch);
	cpdma_ctlr_destroy(priv->dma);
clean_iomap_ret:
	iounmap(priv->regs);
clean_iores_ret:
	release_mem_region(priv->res->start, resource_size(priv->res));
clean_clk_ret:
	clk_put(priv->clk);
clean_slaves_ret:
	kfree(priv->slaves);
clean_ndev_ret:
	free_netdev(ndev);
	return ret;
}

static int __devexit cpsw_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct cpsw_priv *priv = netdev_priv(ndev);

	msg(notice, probe, "removing device\n");
	platform_set_drvdata(pdev, NULL);

	free_irq(ndev->irq, priv);
	cpsw_ale_destroy(priv->ale);
	cpdma_chan_destroy(priv->txch);
	cpdma_chan_destroy(priv->rxch);
	cpdma_ctlr_destroy(priv->dma);
	iounmap(priv->regs);
	release_mem_region(priv->res->start, resource_size(priv->res));
	clk_put(priv->clk);
	kfree(priv->slaves);
	free_netdev(ndev);

	return 0;
}

static int cpsw_suspend(struct device *dev)
{
	struct platform_device	*pdev = to_platform_device(dev);
	struct net_device	*ndev = platform_get_drvdata(pdev);

	if (netif_running(ndev))
		cpsw_ndo_stop(ndev);
	return 0;
}

static int cpsw_resume(struct device *dev)
{
	struct platform_device	*pdev = to_platform_device(dev);
	struct net_device	*ndev = platform_get_drvdata(pdev);

	if (netif_running(ndev))
		cpsw_ndo_open(ndev);
	return 0;
}

static const struct dev_pm_ops cpsw_pm_ops = {
	.suspend	= cpsw_suspend,
	.resume		= cpsw_resume,
};

static struct platform_driver cpsw_driver = {
	.driver = {
		.name	 = "cpsw",
		.owner	 = THIS_MODULE,
		.pm	 = &cpsw_pm_ops,
	},
	.probe = cpsw_probe,
	.remove = __devexit_p(cpsw_remove),
};

static int __init cpsw_init(void)
{
	return platform_driver_register(&cpsw_driver);
}
late_initcall(cpsw_init);

static void __exit cpsw_exit(void)
{
	platform_driver_unregister(&cpsw_driver);
}
module_exit(cpsw_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("TI CPSW Ethernet driver");
