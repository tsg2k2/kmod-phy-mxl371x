// SPDX-License-Identifier: GPL-2.0+
/*
 * Driver for MaxLinear MXL371x MoCA 2.5 PHYs
 *
 * Copyright (c) 2025 Kenneth Kasilag <kenneth@kasilag.me>
 *
 */

#include <linux/module.h>
#include <linux/phy.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/hwmon.h>
#include <linux/random.h>
#include <linux/elf.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/minmax.h>

#include "mxl371x_payloads.h"

/* MaxLinear OUI and PHY IDs */
#define MXL371X_OUI			0x0243E000
#define MXL371X_OUI_MASK		0xFFFFF000

#define MXL3710_PHY_ID			0x02434770
#define MXL3711_PHY_ID			0x02434771

/* Firmware files */
#define MXL371X_FW_LEUCADIA		"ccpu.elf.leucadia"
#define MXL371X_FW_CARDIFF		"ccpu.elf.cardiff"
#define MXL371X_MAX_FW_SIZE		(4 * 1024 * 1024)

/* MoCA SoC Chip Types */
#define MXL_MOCA_SOC_TYPE_LEUCADIA	0
#define MXL_MOCA_SOC_TYPE_CARDIFF	1

/* Standard PHY Registers */
#define MXL371X_BMCR			0x00
#define MXL371X_BMSR			0x01
#define MXL371X_PAGE_SELECT		0x1f

/* System Resource Engine (SRE) Registers */
#define SRE_PRODUCT_FAMILY_ID		0x08200000
#define SRE_DEVICE_ID			0x08200004
#define SRE_REVISION_ID_OFFSET		16
#define SRE_CPU_SRC_SEL_CSR		0x08200010

/* Temperature Sensor Registers */
#define MXL371X_TSENS_CTRL_REG		0x08200200
#define MXL371X_TSENS_DATA_REG		0x08200204
#define MXL371X_RADIO_TSENS_REG1	0x0c14c110
#define MXL371X_RADIO_TSENS_REG2	0x0c14c100
#define MXL371X_RADIO_TSENS_REG3	0x0c14c108

/* Temperature calculation constants */
#define MXL371X_TSENS_COEFF_A		1338680
#define MXL371X_TSENS_COEFF_B		277770
#define MXL371X_TSENS_RSSI_MAX		524288

/* Firmware Status */
#define MXL371X_FW_STATUS_REG		0x08200100
#define MXL371X_FW_LOADED		BIT(0)
#define MXL371X_FW_RUNNING		BIT(1)
#define MXL371X_FW_ERROR		BIT(2)

/*
 * Mailbox boot handshake: the host writes a request magic to HOST_MBOX and a
 * *running* SoC firmware echoes an ack magic.  This is the reliable "is the
 * CPU alive" test -- FW_STATUS_REG above is not a dependable running indicator
 * on this part.
 */
#define MXL371X_HOST_MBOX		0x0c4003a0
#define MXL371X_MBOX_REQ		0x11223344
#define MXL371X_MBOX_ACK		0x55667788	/* hosted boot */
#define MXL371X_MBOX_ACK2		0xaabbccdd	/* nohost boot */
/*
 * Leucadia "boot kick": for socType 0x3710 the host writes 0 here (after the
 * request magic) to release the comms-CPU so it starts executing the
 * downloaded image and begins answering on HOST_MBOX.  This is the step that
 * actually starts the firmware -- the HW/SW resets only prepare for download.
 */
#define MXL371X_BOOT_KICK_REG		0x0c107804
/*
 * Second Leucadia boot register: the host writes 0xa here right after
 * BOOT_KICK_REG to release the comms-CPU.  (For non-Leucadia parts this is a
 * read-modify-write clearing bit0; for 0x3710 the value is the constant 0xa.)
 */
#define MXL371X_BOOT_RUN_REG		0x0c107004
#define MXL371X_BOOT_RUN_VAL		0x0000000a

/*
 * Mailbox command protocol.  After the boot
 * handshake the firmware publishes a layout word at HOST_MBOX+4 describing the
 * command/response region.  All addresses are derived from it:
 *   cmd-id    @ HOST_MBOX+8   (low16 = cmd id, high16 = cmd length in bytes)
 *   cmd-start @ HOST_MBOX+12  (command payload words)
 *   rsp-res   @ (layout>>16)+HOST_MBOX  (low16 = status, high16 = rsp length)
 *   rsp-start = 8-byte-aligned-up(rsp-res + 4)
 * To send: write payload -> cmd-start, write cmd-id word, ring the doorbell,
 * then poll rsp-res until status != 0.  Response status: 1 = success,
 * 2 = unknown command, 0xffff = bus error, 0 = not ready yet.
 */
#define MXL371X_MBOX_LAYOUT_REG		0x0c4003a4
#define MXL371X_MBOX_CMDID_REG		0x0c4003a8
#define MXL371X_MBOX_CMDSTART_REG	0x0c4003ac
/*
 * Doorbell that wakes the SoC after a command is published.  This value is
 * correct for every MDIO-attached part this driver handles (Leucadia and
 * Cardiff are both 0x37xx socTypes).  The alternate 0x0c100870 doorbell
 * applies only to socType 0x27, a PCIe-attached variant not driven over this
 * MDIO interface.
 */
#define MXL371X_MBOX_DOORBELL_REG	0x0c107070
#define MXL371X_MBOX_DOORBELL_VAL	0x00002000

#define MXL371X_MBOX_RSP_NONE		0x0000	/* SoC has not answered yet */
#define MXL371X_MBOX_RSP_OK		0x0001
#define MXL371X_MBOX_RSP_UNKNOWN_CMD	0x0002
#define MXL371X_MBOX_RSP_BUS_ERR	0xffff
/*
 * Mailbox response poll budgets (x10ms).  Boot/config commands can take
 * several seconds, so they get the long budget; the periodic status poll uses
 * a shorter one and simply retries next cycle.
 */
#define MXL371X_MBOX_POLL_STATUS	200	/* 2s  */
#define MXL371X_MBOX_POLL_BOOT		800	/* 8s */

/* SoC command ids (32-bit driver id; low 16 bits go on the wire). */
#define MXL_MOCA_CMD_GET_IMAGE_INFO	0x1010002
#define MXL_MOCA_IMAGE_INFO_RSP_LEN	0x50	/* bytes; socVerStr at +0x38 */

/*
 * Status-poll commands used by the periodic monitor.  All are zero-payload and
 * need no configuration.  Response fields are plain 32-bit values (only the
 * version string needs byte-swapping).
 */
#define MXL_MOCA_CMD_GET_SOC_STATUS	0x1010001
#define MXL_MOCA_SOC_STATUS_RSP_LEN	0x18
#define MXL_MOCA_SOC_STATUS_OFF_STATUS	0x00	/* 1 = healthy */
#define MXL_MOCA_SOC_STATUS_OFF_FATAL	0x04

/* GET_LOCAL_INFO response layout (byte offsets within the response) */
#define MXL_MOCA_CMD_GET_LOCAL_INFO	0x1010015
#define MXL_MOCA_LOCAL_INFO_RSP_LEN	0x74
#define MXL_MOCA_LI_OFF_NODE_ID		0x00
#define MXL_MOCA_LI_OFF_NC_NODE_ID	0x04
#define MXL_MOCA_LI_OFF_LINKSTATUS	0x14	/* 1 = MoCA link up */
#define MXL_MOCA_LI_OFF_ADM		0x18	/* admission status */
#define MXL_MOCA_LI_OFF_NETSTATE	0x24
#define MXL_MOCA_LI_OFF_MOCAVER		0x2c
#define MXL_MOCA_LI_OFF_ACTIVEMASK	0x30

#define MXL_MOCA_CMD_GET_LOF		0x1010024
#define MXL_MOCA_LOF_RSP_LEN		0x08
#define MXL_MOCA_LOF_OFF_LOF		0x04

/*
 * Boot-config sequence: four parameter payloads followed by InitStart.  The
 * payloads are static (in mxl371x_payloads.h); only InitParamV2's GUID is
 * patched per device.  InitStart then admits the node to the network.
 */
#define MXL_MOCA_CMD_INIT_PARAM_V2	0x1010004
#define MXL_MOCA_CMD_RLAPM_V2		0x1010005
#define MXL_MOCA_CMD_SAPM_V2		0x1010006
#define MXL_MOCA_CMD_INIT_PARAM_V25	0x101000a
#define MXL_MOCA_CMD_INIT_START		0x1010007
#define MXL_MOCA_INIT_START_RSP_LEN	0x10

/*
 * MDIO indirect SoC-memory access window (Clause-22 registers).  An earlier
 * 0x0e/0x0f scheme was wrong on this part (those registers are read-only
 * constants, so reads returned garbage and writes were dropped).  The correct
 * window is:
 *   0x1b cmd/status, 0x1c/0x1d address hi/lo, 0x1e/0x1f data hi/lo.
 * Every access is: wait-ready -> set address -> issue command -> wait-ready.
 */
#define MXL371X_MDIO_CMD		0x1b	/* command / status */
#define MXL371X_MDIO_ADDR_HI		0x1c
#define MXL371X_MDIO_ADDR_LO		0x1d
#define MXL371X_MDIO_DATA_HI		0x1e
#define MXL371X_MDIO_DATA_LO		0x1f

#define MXL371X_CMD_STATUS_MASK		0xff	/* ready when 0; 0xff = error */
#define MXL371X_CMD_WRITE		0x1
#define MXL371X_CMD_READ		0x2
#define MXL371X_CMD_TURBO_WRITE		0x5	/* auto-increment write */
#define MXL371X_CMD_TURBO_READ		0x6	/* auto-increment read */
#define MXL371X_CMD_CLOSE		0x0
#define MXL371X_CMD_READY_RETRIES	1000

/* SoC reset + firmware load window (Leucadia software-reset path). */
#define MXL371X_SOC_SWRESET_REG		0x0c106030
#define MXL371X_SOC_SWRESET_VAL		0x40000000
#define MXL371X_SOC_SWRESET_DONE_REG	0x0c106034	/* reads 0x80000000 when reset took */
#define MXL371X_SOC_SWRESET_DONE_VAL	0x80000000
#define MXL371X_FW_LOAD_BASE		0x0c400000
#define MXL371X_FW_LOAD_MASK		0x003fffff
#define MXL371X_FW_CHUNK_BYTES		4096

/* SGMII/HSGMII Configuration */
#define MXL371X_SGMII_CTRL		0xa000
#define MXL371X_SGMII_MODE_MASK		0xff
#define MXL371X_SGMII_MODE_SGMII	0x02
#define MXL371X_SGMII_MODE_HSGMII	0x03
#define MXL371X_SGMII_MODE_1000BASE_X	0x04

/* MoCA Statistics Registers */
#define MOCA_STATS_BASE			0x0c000000
#define MOCA_STATS_TX_TOTAL_PKTS	(MOCA_STATS_BASE + 0x00)
#define MOCA_STATS_TX_TOTAL_BYTES	(MOCA_STATS_BASE + 0x08)
#define MOCA_STATS_TX_DROPPED_PKTS	(MOCA_STATS_BASE + 0x10)
#define MOCA_STATS_TX_BCAST_PKTS	(MOCA_STATS_BASE + 0x18)
#define MOCA_STATS_TX_MCAST_PKTS	(MOCA_STATS_BASE + 0x20)
#define MOCA_STATS_RX_TOTAL_PKTS	(MOCA_STATS_BASE + 0x28)
#define MOCA_STATS_RX_TOTAL_BYTES	(MOCA_STATS_BASE + 0x30)
#define MOCA_STATS_RX_DROPPED_PKTS	(MOCA_STATS_BASE + 0x38)
#define MOCA_STATS_RX_ERROR_PKTS	(MOCA_STATS_BASE + 0x40)

/* MoCA Link Status */
#define MOCA_LINK_STATUS_REG		0x0c100000
#define MOCA_LINK_STATUS_MASK		0x07
#define MOCA_LINK_PHY_RATE_REG		0x0c100004
#define MOCA_LINK_MOCA_VER_REG		0x0c100008
#define MOCA_LINK_NODE_ID_REG		0x0c10000c
#define MOCA_LINK_NC_NODE_ID_REG	0x0c100010
#define MOCA_LINK_LOF_REG		0x0c100014
#define MOCA_LINK_NETWORK_STATE_REG	0x0c100018
#define MOCA_LINK_ACTIVE_NODES_REG	0x0c10001c

/* MoCA Link States */
#define MOCA_LINK_DOWN			0
#define MOCA_LINK_UP			1
#define MOCA_LINK_SCANNING		2

/* MoCA Version */
#define MOCA_VER_1_1			0x11
#define MOCA_VER_2_0			0x20
#define MOCA_VER_2_5			0x25

/* MoCA Network States */
#define MOCA_NET_STATE_IDLE		0
#define MOCA_NET_STATE_SEARCHING	1
#define MOCA_NET_STATE_NETWORK_MODE	2

/* MoCA MAC Address Registers (GUID) */
#define MOCA_MAC_ADDR_HI		0x0c100020
#define MOCA_MAC_ADDR_LO		0x0c100024

/* Privacy/Security Status */
#define MOCA_SECURITY_STATUS_REG	0x0c100200
#define MOCA_SECURITY_ENABLED		BIT(0)

struct mxl371x_priv {
	struct phy_device *phydev;	/* back-pointer for the stats workqueue */
	bool fw_loaded;
	u32 soc_chip_type;
	u32 device_id;
	u32 revision_id;
	u32 link_status;
	u32 moca_version;
	u32 phy_rate;
	u32 node_id;
	u32 nc_node_id;
	u32 lof;
	u32 network_state;
	u32 active_nodes;
	bool security_enabled;
	const char *fw_name;
	char soc_version[64];
	char fw_version[64];	/* real firmware build string, via mailbox */

	/* Mailbox command region (computed after boot from the layout word) */
	bool mbox_ready;
	struct mutex mbox_lock;	/* serializes whole mailbox transactions */
	u32 mbox_cmdid_addr;
	u32 mbox_cmdstart_addr;
	u32 mbox_rsp_res_addr;
	u32 mbox_rsp_start_addr;
	u32 mbox_cmd_byte_max;

	/* debug: live mailbox command probe (write cmd-id -> read raw rsp hex) */
	u16 dbg_cmd;
	int dbg_rsp_len;	/* bytes returned by last probe, or -errno */
	u32 dbg_rsp[384 / 4];

	/* Statistics */
	struct {
		u64 tx_packets;
		u64 tx_bytes;
		u64 tx_dropped;
		u64 tx_broadcast;
		u64 tx_multicast;
		u64 rx_packets;
		u64 rx_bytes;
		u64 rx_dropped;
		u64 rx_errors;
	} stats;

	struct delayed_work stats_poll;
	struct device *hwmon_dev;
};

static int mxl371x_read_page(struct phy_device *phydev)
{
	return __phy_read(phydev, MXL371X_PAGE_SELECT);
}

static int mxl371x_write_page(struct phy_device *phydev, int page)
{
	return __phy_write(phydev, MXL371X_PAGE_SELECT, page);
}

/*
 * Poll the command/status register (0x1b) until the access engine is idle.
 * Must be called with the MDIO bus already locked (busy bits clear when the
 * low byte reads 0; 0xff = error).
 */
static int mxl371x_wait_ready(struct phy_device *phydev)
{
	int i, val;

	for (i = 0; i < MXL371X_CMD_READY_RETRIES; i++) {
		val = __phy_read(phydev, MXL371X_MDIO_CMD);
		if (val < 0)
			return val;
		if ((val & MXL371X_CMD_STATUS_MASK) == MXL371X_CMD_STATUS_MASK)
			return -EIO;
		if ((val & MXL371X_CMD_STATUS_MASK) == 0)
			return 0;
	}
	return -ETIMEDOUT;
}

static int mxl371x_read_mem32(struct phy_device *phydev, u32 addr, u32 *val)
{
	int ret, hi, lo;

	phy_lock_mdio_bus(phydev);

	ret = mxl371x_wait_ready(phydev);
	if (ret < 0)
		goto out;

	ret = __phy_write(phydev, MXL371X_MDIO_ADDR_HI, (addr >> 16) & 0xffff);
	if (!ret)
		ret = __phy_write(phydev, MXL371X_MDIO_ADDR_LO, addr & 0xffff);
	if (!ret)
		ret = __phy_write(phydev, MXL371X_MDIO_CMD, MXL371X_CMD_READ);
	if (ret < 0)
		goto out;

	ret = mxl371x_wait_ready(phydev);
	if (ret < 0)
		goto out;

	hi = __phy_read(phydev, MXL371X_MDIO_DATA_HI);
	lo = __phy_read(phydev, MXL371X_MDIO_DATA_LO);
	if (hi < 0 || lo < 0) {
		ret = -EIO;
		goto out;
	}

	*val = ((u32)(hi & 0xffff) << 16) | (lo & 0xffff);
	ret = 0;
out:
	phy_unlock_mdio_bus(phydev);
	return ret;
}

static int mxl371x_read_mem64(struct phy_device *phydev, u32 addr, u64 *val)
{
	u32 val_lo, val_hi;
	int ret;

	ret = mxl371x_read_mem32(phydev, addr, &val_lo);
	if (ret < 0)
		return ret;

	ret = mxl371x_read_mem32(phydev, addr + 4, &val_hi);
	if (ret < 0)
		return ret;

	*val = ((u64)val_hi << 32) | val_lo;
	return 0;
}

static int mxl371x_write_mem32(struct phy_device *phydev, u32 addr, u32 val)
{
	int ret;

	phy_lock_mdio_bus(phydev);

	ret = mxl371x_wait_ready(phydev);
	if (ret < 0)
		goto out;

	ret = __phy_write(phydev, MXL371X_MDIO_ADDR_HI, (addr >> 16) & 0xffff);
	if (!ret)
		ret = __phy_write(phydev, MXL371X_MDIO_ADDR_LO, addr & 0xffff);
	if (!ret)
		ret = __phy_write(phydev, MXL371X_MDIO_DATA_HI, (val >> 16) & 0xffff);
	if (!ret)
		ret = __phy_write(phydev, MXL371X_MDIO_DATA_LO, val & 0xffff);
	if (!ret)
		ret = __phy_write(phydev, MXL371X_MDIO_CMD, MXL371X_CMD_WRITE);
out:
	phy_unlock_mdio_bus(phydev);
	return ret;
}

/*
 * Stream a block of 32-bit words to consecutive SoC addresses using the MDIO
 * "turbo" auto-increment mode: latch the base address once, then push word
 * after word.  Words must already be in SoC byte order.  Used for the bulk
 * firmware upload.
 */
static int mxl371x_turbo_write_block(struct phy_device *phydev, u32 addr,
				     const u32 *words, u32 count)
{
	int ret;
	u32 i;

	phy_lock_mdio_bus(phydev);

	ret = mxl371x_wait_ready(phydev);
	if (ret < 0)
		goto out;

	/* Turbo open: latch base address (low half is written twice). */
	ret = __phy_write(phydev, MXL371X_MDIO_ADDR_HI, (addr >> 16) & 0xffff);
	if (!ret)
		ret = __phy_write(phydev, MXL371X_MDIO_ADDR_LO, addr & 0xffff);
	if (!ret)
		ret = __phy_write(phydev, MXL371X_MDIO_ADDR_LO, addr & 0xffff);
	if (ret < 0)
		goto out;

	for (i = 0; i < count; i++) {
		ret = __phy_write(phydev, MXL371X_MDIO_DATA_HI,
				  (words[i] >> 16) & 0xffff);
		if (!ret)
			ret = __phy_write(phydev, MXL371X_MDIO_DATA_LO,
					  words[i] & 0xffff);
		if (!ret)
			ret = __phy_write(phydev, MXL371X_MDIO_CMD,
					  MXL371X_CMD_TURBO_WRITE);
		if (ret < 0)
			break;
	}

	/* Turbo close (best effort). */
	__phy_write(phydev, MXL371X_MDIO_CMD, MXL371X_CMD_CLOSE);
out:
	phy_unlock_mdio_bus(phydev);
	return ret;
}

/* Temperature sensor reading */
static int mxl371x_read_temp_raw(struct phy_device *phydev, u32 *t0, u32 *t1)
{
	int ret;

	/* Get T0 reading */
	ret = mxl371x_write_mem32(phydev, MXL371X_RADIO_TSENS_REG1, 0x31000001);
	if (ret < 0)
		return ret;

	ret = mxl371x_write_mem32(phydev, MXL371X_RADIO_TSENS_REG2, 0x00000401);
	if (ret < 0)
		return ret;

	ret = mxl371x_write_mem32(phydev, MXL371X_RADIO_TSENS_REG3, 0x00000001);
	if (ret < 0)
		return ret;

	ret = mxl371x_write_mem32(phydev, MXL371X_TSENS_CTRL_REG, 0x01130103);
	if (ret < 0)
		return ret;

	usleep_range(30000, 40000);

	ret = mxl371x_read_mem32(phydev, MXL371X_TSENS_DATA_REG, t0);
	if (ret < 0)
		return ret;

	/* Get T1 reading */
	ret = mxl371x_write_mem32(phydev, MXL371X_TSENS_CTRL_REG, 0x01130003);
	if (ret < 0)
		return ret;

	ret = mxl371x_write_mem32(phydev, MXL371X_RADIO_TSENS_REG2, 0x00000411);
	if (ret < 0)
		return ret;

	ret = mxl371x_write_mem32(phydev, MXL371X_TSENS_CTRL_REG, 0x01130003);
	if (ret < 0)
		return ret;

	ret = mxl371x_write_mem32(phydev, MXL371X_TSENS_CTRL_REG, 0x01130103);
	if (ret < 0)
		return ret;

	usleep_range(30000, 40000);

	ret = mxl371x_read_mem32(phydev, MXL371X_TSENS_DATA_REG, t1);
	if (ret < 0)
		return ret;

	return 0;
}

static int mxl371x_calc_temp(u32 t0, u32 t1)
{
	s64 delta, temp;

	if (t1 < t0)
		return -EINVAL;

	delta = (s64)(t1 - t0);
	temp = (delta * MXL371X_TSENS_COEFF_A) / MXL371X_TSENS_RSSI_MAX;
	temp -= MXL371X_TSENS_COEFF_B;

	return (int)temp;
}

/* Update statistics from hardware */
static void mxl371x_update_stats(struct phy_device *phydev)
{
	struct mxl371x_priv *priv = phydev->priv;
	struct device *dev = &phydev->mdio.dev;
	int ret;

	ret = mxl371x_read_mem64(phydev, MOCA_STATS_TX_TOTAL_PKTS,
				 &priv->stats.tx_packets);
	ret |= mxl371x_read_mem64(phydev, MOCA_STATS_TX_TOTAL_BYTES,
				  &priv->stats.tx_bytes);
	ret |= mxl371x_read_mem64(phydev, MOCA_STATS_TX_DROPPED_PKTS,
				  &priv->stats.tx_dropped);
	ret |= mxl371x_read_mem64(phydev, MOCA_STATS_TX_BCAST_PKTS,
				  &priv->stats.tx_broadcast);
	ret |= mxl371x_read_mem64(phydev, MOCA_STATS_TX_MCAST_PKTS,
				  &priv->stats.tx_multicast);
	ret |= mxl371x_read_mem64(phydev, MOCA_STATS_RX_TOTAL_PKTS,
				  &priv->stats.rx_packets);
	ret |= mxl371x_read_mem64(phydev, MOCA_STATS_RX_TOTAL_BYTES,
				  &priv->stats.rx_bytes);
	ret |= mxl371x_read_mem64(phydev, MOCA_STATS_RX_DROPPED_PKTS,
				  &priv->stats.rx_dropped);
	ret |= mxl371x_read_mem64(phydev, MOCA_STATS_RX_ERROR_PKTS,
				  &priv->stats.rx_errors);

	if (ret < 0)
		dev_warn_ratelimited(dev, "Failed to update MoCA statistics\n");
}

static int mxl371x_mbox_cmd(struct phy_device *phydev, u16 cmd_id,
			    const u32 *payload, u32 payload_len,
			    u32 *rsp, u32 rsp_max_words, u32 poll_retries);

/*
 * Update MoCA link state over the mailbox.  An earlier approach read the
 * 0x0c1000xx link-state registers directly, but on running firmware those
 * return "engine busy" -- live state is only available through mailbox
 * commands.
 *
 * GET_LOCAL_INFO carries the link-up flag (and admission status); GET_LOF gives
 * the last operating frequency once the link is up.  Returns 0 on success.
 */
static int mxl371x_read_moca_status(struct phy_device *phydev)
{
	struct mxl371x_priv *priv = phydev->priv;
	u32 rsp[MXL_MOCA_LOCAL_INFO_RSP_LEN / 4];
	int ret;

	if (!priv->mbox_ready)
		return -ENODEV;

	ret = mxl371x_mbox_cmd(phydev, MXL_MOCA_CMD_GET_LOCAL_INFO & 0xffff,
			       NULL, 0, rsp, ARRAY_SIZE(rsp),
			       MXL371X_MBOX_POLL_STATUS);
	if (ret < MXL_MOCA_LOCAL_INFO_RSP_LEN)
		return ret < 0 ? ret : -EIO;

	priv->link_status = (rsp[MXL_MOCA_LI_OFF_LINKSTATUS / 4] == 1) ?
			    MOCA_LINK_UP : MOCA_LINK_DOWN;
	priv->node_id      = rsp[MXL_MOCA_LI_OFF_NODE_ID / 4];
	priv->nc_node_id   = rsp[MXL_MOCA_LI_OFF_NC_NODE_ID / 4];
	priv->network_state = rsp[MXL_MOCA_LI_OFF_NETSTATE / 4];
	priv->moca_version = rsp[MXL_MOCA_LI_OFF_MOCAVER / 4];
	priv->active_nodes = rsp[MXL_MOCA_LI_OFF_ACTIVEMASK / 4];

	/* LOF is only meaningful once the link is up. */
	if (priv->link_status == MOCA_LINK_UP) {
		u32 lof[MXL_MOCA_LOF_RSP_LEN / 4];

		if (mxl371x_mbox_cmd(phydev, MXL_MOCA_CMD_GET_LOF & 0xffff,
				     NULL, 0, lof, ARRAY_SIZE(lof),
				     MXL371X_MBOX_POLL_STATUS) >=
		    MXL_MOCA_LOF_RSP_LEN)
			priv->lof = lof[MXL_MOCA_LOF_OFF_LOF / 4];
	}

	return 0;
}

static void mxl371x_stats_poll_work(struct work_struct *work)
{
	struct mxl371x_priv *priv = container_of(work, struct mxl371x_priv,
						 stats_poll.work);
	struct phy_device *phydev = priv->phydev;
	unsigned long delay = HZ;

	/*
	 * Re-read the live MoCA link state over the mailbox.  This must run
	 * regardless of attached_dev (a DSA/phylink port may not set it): it is
	 * the only path that keeps link_status current after the initial probe
	 * read.  Stats need the netdev, so they stay gated on attached_dev.
	 */
	if (priv->mbox_ready) {
		if (mxl371x_read_moca_status(phydev) < 0)
			delay = 30 * HZ;
		else if (phydev->attached_dev)
			mxl371x_update_stats(phydev);
	}

	schedule_delayed_work(&priv->stats_poll, delay);
}

/* Standard ethtool PHY statistics */
static void mxl371x_get_phy_stats(struct phy_device *phydev,
				  struct ethtool_eth_phy_stats *phy_stats,
				  struct ethtool_phy_stats *phydev_stats)
{
	struct mxl371x_priv *priv = phydev->priv;

	phydev_stats->rx_packets = priv->stats.rx_packets;
	phydev_stats->rx_bytes = priv->stats.rx_bytes;
	phydev_stats->rx_errors = priv->stats.rx_errors;
	phydev_stats->tx_packets = priv->stats.tx_packets;
	phydev_stats->tx_bytes = priv->stats.tx_bytes;
	phydev_stats->tx_errors = priv->stats.tx_dropped;
}

/* HWMON temperature sensor support */
static int mxl371x_hwmon_read(struct device *dev, enum hwmon_sensor_types type,
			      u32 attr, int channel, long *val)
{
	struct phy_device *phydev = dev_get_drvdata(dev);
	u32 t0, t1;
	int ret, temp;

	if (type != hwmon_temp)
		return -EOPNOTSUPP;

	switch (attr) {
	case hwmon_temp_input:
		ret = mxl371x_read_temp_raw(phydev, &t0, &t1);
		if (ret < 0)
			return ret;

		temp = mxl371x_calc_temp(t0, t1);
		if (temp == -EINVAL)
			return -EINVAL;

		*val = temp;
		break;

	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static umode_t mxl371x_hwmon_is_visible(const void *data,
					enum hwmon_sensor_types type,
					u32 attr, int channel)
{
	if (type != hwmon_temp)
		return 0;

	switch (attr) {
	case hwmon_temp_input:
		return 0444;
	default:
		return 0;
	}
}

static const struct hwmon_ops mxl371x_hwmon_ops = {
	.is_visible = mxl371x_hwmon_is_visible,
	.read = mxl371x_hwmon_read,
};

static const struct hwmon_channel_info *mxl371x_hwmon_info[] = {
	HWMON_CHANNEL_INFO(temp, HWMON_T_INPUT),
	NULL
};

static const struct hwmon_chip_info mxl371x_hwmon_chip_info = {
	.ops = &mxl371x_hwmon_ops,
	.info = mxl371x_hwmon_info,
};

static int mxl371x_hwmon_init(struct phy_device *phydev)
{
	struct mxl371x_priv *priv = phydev->priv;
	struct device *dev = &phydev->mdio.dev;
	struct device *hwmon_dev;

	hwmon_dev = devm_hwmon_device_register_with_info(dev, "mxl371x",
							 phydev,
							 &mxl371x_hwmon_chip_info,
							 NULL);
	if (IS_ERR(hwmon_dev))
		return PTR_ERR(hwmon_dev);

	priv->hwmon_dev = hwmon_dev;
	return 0;
}

/* Check if firmware is already running (warm boot) */
static int mxl371x_check_firmware_running(struct phy_device *phydev)
{
	struct device *dev = &phydev->mdio.dev;
	u32 fw_status;
	int ret;

	ret = mxl371x_read_mem32(phydev, MXL371X_FW_STATUS_REG, &fw_status);
	if (ret < 0) {
		dev_warn(dev, "Cannot read firmware status: %d\n", ret);
		return 0;
	}

	if (fw_status & MXL371X_FW_RUNNING) {
		dev_info(dev, "Firmware already running (warm boot detected)\n");
		return 1;
	}

	if (fw_status & MXL371X_FW_ERROR) {
		dev_warn(dev, "Firmware in error state, will reload\n");
		return 0;
	}

	return 0;
}

/*
 * Derive the MoCA GUID (as the SoC eMacAddrHi/eMacAddrLo word pair) from, in
 * order: the device tree, the attached netdev MAC, or a random MaxLinear-OUI
 * address.  These two words are patched into the InitParamV2 config payload.
 */
static void mxl371x_derive_guid(struct phy_device *phydev, u32 *mac_hi,
				u32 *mac_lo)
{
	struct device *dev = &phydev->mdio.dev;
	u8 mac[ETH_ALEN];

	if (dev->of_node && of_get_mac_address(dev->of_node, mac) == 0) {
		dev_info(dev, "MoCA GUID from device tree: %pM\n", mac);
	} else if (phydev->attached_dev &&
		   !is_zero_ether_addr(phydev->attached_dev->dev_addr)) {
		ether_addr_copy(mac, phydev->attached_dev->dev_addr);
		mac[0] |= 0x02;		/* locally administered */
		mac[5] ^= 0x01;		/* distinguish from host MAC */
		dev_info(dev, "MoCA GUID from netdev: %pM\n", mac);
	} else {
		mac[0] = 0x02;
		mac[1] = 0x24;
		mac[2] = 0x3e;		/* MaxLinear OUI */
		get_random_bytes(&mac[3], 3);
		dev_info(dev, "MoCA GUID generated: %pM\n", mac);
	}

	*mac_hi = (mac[0] << 24) | (mac[1] << 16) | (mac[2] << 8) | mac[3];
	*mac_lo = (mac[4] << 24) | (mac[5] << 16);
}

/* Sysfs attributes */
static ssize_t moca_link_status_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct phy_device *phydev = to_phy_device(dev);
	struct mxl371x_priv *priv = phydev->priv;
	const char *status;

	switch (priv->link_status) {
	case MOCA_LINK_UP:
		status = "up";
		break;
	case MOCA_LINK_SCANNING:
		status = "scanning";
		break;
	default:
		status = "down";
		break;
	}

	return sprintf(buf, "%s\n", status);
}
static DEVICE_ATTR_RO(moca_link_status);

static ssize_t moca_version_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct phy_device *phydev = to_phy_device(dev);
	struct mxl371x_priv *priv = phydev->priv;

	return sprintf(buf, "%u.%u\n", priv->moca_version >> 4,
		       priv->moca_version & 0xf);
}
static DEVICE_ATTR_RO(moca_version);

static ssize_t moca_phy_rate_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct phy_device *phydev = to_phy_device(dev);
	struct mxl371x_priv *priv = phydev->priv;

	return sprintf(buf, "%u\n", priv->phy_rate);
}
static DEVICE_ATTR_RO(moca_phy_rate);

static ssize_t moca_node_id_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct phy_device *phydev = to_phy_device(dev);
	struct mxl371x_priv *priv = phydev->priv;

	return sprintf(buf, "%u\n", priv->node_id);
}
static DEVICE_ATTR_RO(moca_node_id);

static ssize_t moca_nc_node_id_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct phy_device *phydev = to_phy_device(dev);
	struct mxl371x_priv *priv = phydev->priv;

	return sprintf(buf, "%u\n", priv->nc_node_id);
}
static DEVICE_ATTR_RO(moca_nc_node_id);

static ssize_t moca_lof_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct phy_device *phydev = to_phy_device(dev);
	struct mxl371x_priv *priv = phydev->priv;

	return sprintf(buf, "%u\n", priv->lof);
}
static DEVICE_ATTR_RO(moca_lof);

static ssize_t moca_network_state_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct phy_device *phydev = to_phy_device(dev);
	struct mxl371x_priv *priv = phydev->priv;
	const char *state;

	switch (priv->network_state) {
	case MOCA_NET_STATE_NETWORK_MODE:
		state = "network";
		break;
	case MOCA_NET_STATE_SEARCHING:
		state = "searching";
		break;
	default:
		state = "idle";
		break;
	}

	return sprintf(buf, "%s\n", state);
}
static DEVICE_ATTR_RO(moca_network_state);

static ssize_t moca_active_nodes_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct phy_device *phydev = to_phy_device(dev);
	struct mxl371x_priv *priv = phydev->priv;

	return sprintf(buf, "0x%08x\n", priv->active_nodes);
}
static DEVICE_ATTR_RO(moca_active_nodes);

static ssize_t moca_security_enabled_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct phy_device *phydev = to_phy_device(dev);
	struct mxl371x_priv *priv = phydev->priv;

	return sprintf(buf, "%u\n", priv->security_enabled ? 1 : 0);
}
static DEVICE_ATTR_RO(moca_security_enabled);

static ssize_t moca_chip_type_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct phy_device *phydev = to_phy_device(dev);
	struct mxl371x_priv *priv = phydev->priv;

	return sprintf(buf, "%s\n",
		       priv->soc_chip_type == MXL_MOCA_SOC_TYPE_LEUCADIA ?
		       "leucadia" : "cardiff");
}
static DEVICE_ATTR_RO(moca_chip_type);

static ssize_t moca_fw_version_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct phy_device *phydev = to_phy_device(dev);
	struct mxl371x_priv *priv = phydev->priv;

	/* Prefer the real build string read over the mailbox; fall back to the
	 * device/revision string derived from the SRE registers. */
	if (priv->fw_version[0])
		return sprintf(buf, "%s\n", priv->fw_version);
	return sprintf(buf, "%s\n", priv->soc_version);
}
static DEVICE_ATTR_RO(moca_fw_version);

/* MoCA GUID - read/write */
static ssize_t moca_guid_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct phy_device *phydev = to_phy_device(dev);
	u32 mac_hi, mac_lo;
	u8 mac[ETH_ALEN];

	if (mxl371x_read_mem32(phydev, MOCA_MAC_ADDR_HI, &mac_hi) < 0)
		return -EIO;
	if (mxl371x_read_mem32(phydev, MOCA_MAC_ADDR_LO, &mac_lo) < 0)
		return -EIO;

	mac[0] = (mac_hi >> 24) & 0xff;
	mac[1] = (mac_hi >> 16) & 0xff;
	mac[2] = (mac_hi >> 8) & 0xff;
	mac[3] = (mac_hi >> 0) & 0xff;
	mac[4] = (mac_lo >> 24) & 0xff;
	mac[5] = (mac_lo >> 16) & 0xff;

	return sprintf(buf, "%pM\n", mac);
}

static ssize_t moca_guid_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct phy_device *phydev = to_phy_device(dev);
	u8 mac[ETH_ALEN];
	u32 mac_hi, mac_lo;

	if (!mac_pton(buf, mac))
		return -EINVAL;

	/* Allow any non-zero MAC for MoCA GUID */
	if (is_zero_ether_addr(mac))
		return -EADDRNOTAVAIL;

	mac_hi = (mac[0] << 24) | (mac[1] << 16) | (mac[2] << 8) | mac[3];
	mac_lo = (mac[4] << 24) | (mac[5] << 16);

	if (mxl371x_write_mem32(phydev, MOCA_MAC_ADDR_HI, mac_hi) < 0)
		return -EIO;
	if (mxl371x_write_mem32(phydev, MOCA_MAC_ADDR_LO, mac_lo) < 0)
		return -EIO;

	dev_info(dev, "MoCA GUID set to %pM\n", mac);
	return count;
}
static DEVICE_ATTR_RW(moca_guid);

/*
 * Debug: issue an arbitrary mailbox command and capture the raw response so we
 * can probe firmware status/network queries live without reflashing.
 *   echo 0x15            > moca_dbg_cmd   # GET_LOCAL_INFO, zero payload
 *   echo "0x16 00 00 00 00" > moca_dbg_cmd   # cmd 0x16 with 4 payload bytes
 *   cat moca_dbg_cmd     # raw response words (byte offset + native LE word)
 * First hex token is the wire cmd-id; any further hex tokens are payload bytes
 * (padded up to a 4-byte boundary).
 */
static ssize_t moca_dbg_cmd_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct phy_device *phydev = to_phy_device(dev);
	struct mxl371x_priv *priv = phydev->priv;
	u32 payload[64] = {};
	u8 *pb = (u8 *)payload;
	unsigned int cmd, val, n = 0, plen;
	const char *p = buf;
	int consumed, ret;

	if (!priv->mbox_ready)
		return -ENODEV;
	if (sscanf(p, "%x%n", &cmd, &consumed) < 1)
		return -EINVAL;
	p += consumed;
	while (n < sizeof(payload) && sscanf(p, "%x%n", &val, &consumed) == 1) {
		pb[n++] = val & 0xff;
		p += consumed;
	}
	plen = round_up(n, 4);

	priv->dbg_cmd = cmd & 0xffff;
	ret = mxl371x_mbox_cmd(phydev, priv->dbg_cmd, n ? payload : NULL, plen,
			       priv->dbg_rsp, ARRAY_SIZE(priv->dbg_rsp),
			       MXL371X_MBOX_POLL_STATUS);
	priv->dbg_rsp_len = ret;
	dev_info(&phydev->mdio.dev,
		 "dbg mbox cmd 0x%04x (%u payload bytes) -> %d\n",
		 priv->dbg_cmd, plen, ret);
	return count;
}

static ssize_t moca_dbg_cmd_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct phy_device *phydev = to_phy_device(dev);
	struct mxl371x_priv *priv = phydev->priv;
	int len = priv->dbg_rsp_len;
	int n, i;

	if (len < 0)
		return sprintf(buf, "cmd=0x%04x error=%d\n", priv->dbg_cmd, len);

	if (len > (int)sizeof(priv->dbg_rsp))
		len = sizeof(priv->dbg_rsp);
	n = sprintf(buf, "cmd=0x%04x rsp_bytes=%d\n", priv->dbg_cmd, len);
	for (i = 0; i * 4 < len; i++)
		n += sprintf(buf + n, "[0x%02x] %08x\n", i * 4, priv->dbg_rsp[i]);
	return n;
}
static DEVICE_ATTR_RW(moca_dbg_cmd);

static struct attribute *mxl371x_attrs[] = {
	&dev_attr_moca_link_status.attr,
	&dev_attr_moca_version.attr,
	&dev_attr_moca_phy_rate.attr,
	&dev_attr_moca_node_id.attr,
	&dev_attr_moca_nc_node_id.attr,
	&dev_attr_moca_lof.attr,
	&dev_attr_moca_network_state.attr,
	&dev_attr_moca_active_nodes.attr,
	&dev_attr_moca_security_enabled.attr,
	&dev_attr_moca_chip_type.attr,
	&dev_attr_moca_fw_version.attr,
	&dev_attr_moca_guid.attr,
	&dev_attr_moca_dbg_cmd.attr,
	NULL,
};

static const struct attribute_group mxl371x_attr_group = {
	.attrs = mxl371x_attrs,
};

static int mxl371x_get_device_info(struct phy_device *phydev)
{
	struct mxl371x_priv *priv = phydev->priv;
	struct device *dev = &phydev->mdio.dev;
	u32 val;
	int ret;

	ret = mxl371x_read_mem32(phydev, SRE_PRODUCT_FAMILY_ID, &val);
	if (ret < 0)
		return ret;

	dev_info(dev, "Product Family ID: 0x%08x\n", val);

	ret = mxl371x_read_mem32(phydev, SRE_DEVICE_ID, &val);
	if (ret < 0)
		return ret;

	priv->device_id = val & 0xffff;
	priv->revision_id = (val >> SRE_REVISION_ID_OFFSET) & 0xffff;

	/* SRE_DEVICE_ID may read as 0 before firmware is loaded (indirect
	 * memory access addr_lo can't reach +4 offset pre-boot).  Fall back
	 * to the standard MDIO PHY ID, which the kernel already verified. */
	if (priv->device_id == 0x3710 || priv->device_id == 0x3711 ||
	    phydev->phy_id == MXL3710_PHY_ID || phydev->phy_id == MXL3711_PHY_ID) {
		priv->soc_chip_type = MXL_MOCA_SOC_TYPE_LEUCADIA;
		priv->fw_name = MXL371X_FW_LEUCADIA;
	} else {
		priv->soc_chip_type = MXL_MOCA_SOC_TYPE_CARDIFF;
		priv->fw_name = MXL371X_FW_CARDIFF;
	}

	snprintf(priv->soc_version, sizeof(priv->soc_version),
		 "%s Device 0x%04x Rev 0x%04x",
		 priv->soc_chip_type == MXL_MOCA_SOC_TYPE_LEUCADIA ?
		 "Leucadia" : "Cardiff", priv->device_id, priv->revision_id);

	dev_info(dev, "%s\n", priv->soc_version);

	return 0;
}

/*
 * Finalize the mailbox region after the boot handshake: the running firmware
 * has published a layout word at MBOX_LAYOUT_REG from which
 * the command/response addresses are derived.  Must be called with HOST_MBOX
 * still holding the ack magic; it releases the handshake cell on success.
 */
static int mxl371x_mbox_init(struct phy_device *phydev)
{
	struct mxl371x_priv *priv = phydev->priv;
	struct device *dev = &phydev->mdio.dev;
	u32 layout, hi, rsp_res;
	int ret;

	ret = mxl371x_read_mem32(phydev, MXL371X_MBOX_LAYOUT_REG, &layout);
	if (ret < 0)
		return ret;

	hi = layout >> 16;
	if (hi <= 0xc || hi > 0x4000) {
		dev_warn(dev, "Mailbox layout word invalid (0x%08x)\n", layout);
		return -EIO;
	}

	rsp_res = hi + MXL371X_HOST_MBOX;
	priv->mbox_cmdid_addr     = MXL371X_MBOX_CMDID_REG;
	priv->mbox_cmdstart_addr  = MXL371X_MBOX_CMDSTART_REG;
	priv->mbox_rsp_res_addr   = rsp_res;
	priv->mbox_rsp_start_addr = ALIGN(rsp_res + 4, 8);
	priv->mbox_cmd_byte_max   = hi - 0xc;

	/* Release the handshake cell so the SoC treats the mailbox as idle. */
	ret = mxl371x_write_mem32(phydev, MXL371X_HOST_MBOX, 0);
	if (ret < 0) {
		dev_warn(dev, "Mailbox handshake clear failed: %d\n", ret);
		return ret;
	}
	priv->mbox_ready = true;

	dev_info(dev,
		 "Mailbox ready (layout 0x%08x: cmd@0x%08x rsp@0x%08x/0x%08x max %u)\n",
		 layout, priv->mbox_cmdstart_addr, priv->mbox_rsp_res_addr,
		 priv->mbox_rsp_start_addr, priv->mbox_cmd_byte_max);
	return 0;
}

/*
 * Send one mailbox command and collect its response.  payload/rsp are arrays
 * of 32-bit SoC words.  Returns the response length in bytes (>= 0) or a
 * negative errno.
 */
static int __mxl371x_mbox_cmd(struct phy_device *phydev, u16 cmd_id,
			      const u32 *payload, u32 payload_len,
			      u32 *rsp, u32 rsp_max_words, u32 poll_retries)
{
	struct mxl371x_priv *priv = phydev->priv;
	struct device *dev = &phydev->mdio.dev;
	u32 v = 0, words, i, rsp_len, status;
	int ret;

	if (!priv->mbox_ready)
		return -ENODEV;
	if ((payload_len & 3) || payload_len > priv->mbox_cmd_byte_max)
		return -EINVAL;

	/*
	 * Drain any residue left by a previously abandoned (timed-out) command
	 * so one timeout does not wedge the mailbox.  Holding mbox_lock makes us
	 * the sole host accessor, so a non-zero cmd-id/response here is stale.
	 */
	ret = mxl371x_read_mem32(phydev, priv->mbox_cmdid_addr, &v);
	if (ret < 0)
		return ret;
	if (v != 0)
		mxl371x_write_mem32(phydev, priv->mbox_cmdid_addr, 0);
	ret = mxl371x_read_mem32(phydev, priv->mbox_rsp_res_addr, &v);
	if (ret < 0)
		return ret;
	if (v != 0)
		mxl371x_write_mem32(phydev, priv->mbox_rsp_res_addr, 0);

	/* Write the command payload, publish the id+length, ring the doorbell. */
	words = payload_len / 4;
	for (i = 0; i < words; i++) {
		ret = mxl371x_write_mem32(phydev,
					  priv->mbox_cmdstart_addr + i * 4,
					  payload[i]);
		if (ret < 0)
			return ret;
	}
	ret = mxl371x_write_mem32(phydev, priv->mbox_cmdid_addr,
				  cmd_id | (payload_len << 16));
	if (ret < 0)
		return ret;
	ret = mxl371x_write_mem32(phydev, MXL371X_MBOX_DOORBELL_REG,
				  MXL371X_MBOX_DOORBELL_VAL);
	if (ret < 0)
		return ret;

	for (i = 0; i < poll_retries; i++) {
		ret = mxl371x_read_mem32(phydev, priv->mbox_rsp_res_addr, &v);
		if (ret < 0)
			return ret;
		status = v & 0xffff;
		if (status == MXL371X_MBOX_RSP_NONE) {
			msleep(10);
			continue;
		}
		if (status == MXL371X_MBOX_RSP_OK)
			break;
		mxl371x_write_mem32(phydev, priv->mbox_rsp_res_addr, 0);
		dev_warn(dev, "Mailbox cmd 0x%04x failed (status 0x%04x)\n",
			 cmd_id, status);
		return status == MXL371X_MBOX_RSP_UNKNOWN_CMD ? -EOPNOTSUPP : -EIO;
	}
	if (i == poll_retries) {
		dev_warn(dev, "Mailbox cmd 0x%04x timed out\n", cmd_id);
		/* Abandon cleanly so the next command is not blocked by residue. */
		mxl371x_write_mem32(phydev, priv->mbox_cmdid_addr, 0);
		mxl371x_write_mem32(phydev, priv->mbox_rsp_res_addr, 0);
		return -ETIMEDOUT;
	}

	rsp_len = v >> 16;
	words = rsp_len / 4;
	if (rsp && words) {
		u32 n = min(words, rsp_max_words);

		for (i = 0; i < n; i++) {
			if (mxl371x_read_mem32(phydev,
					       priv->mbox_rsp_start_addr + i * 4,
					       &rsp[i]) < 0)
				break;
		}
	}

	/* Acknowledge by clearing the response-result word. */
	mxl371x_write_mem32(phydev, priv->mbox_rsp_res_addr, 0);
	return rsp_len;
}

/* Serialize whole mailbox transactions: the SoC has a single command slot. */
static int mxl371x_mbox_cmd(struct phy_device *phydev, u16 cmd_id,
			    const u32 *payload, u32 payload_len,
			    u32 *rsp, u32 rsp_max_words, u32 poll_retries)
{
	struct mxl371x_priv *priv = phydev->priv;
	int ret;

	mutex_lock(&priv->mbox_lock);
	ret = __mxl371x_mbox_cmd(phydev, cmd_id, payload, payload_len,
				 rsp, rsp_max_words, poll_retries);
	mutex_unlock(&priv->mbox_lock);
	return ret;
}

/*
 * Query the running firmware build string over the mailbox (GET_IMAGE_INFO,
 * cmd 0x1010002).  Zero-payload, no config required -- this is the first real
 * mailbox round-trip and proves the command path works.
 */
static void mxl371x_query_fw_version(struct phy_device *phydev)
{
	struct mxl371x_priv *priv = phydev->priv;
	struct device *dev = &phydev->mdio.dev;
	u32 rsp[MXL_MOCA_IMAGE_INFO_RSP_LEN / 4] = {};
	int ret, i;

	ret = mxl371x_mbox_cmd(phydev, MXL_MOCA_CMD_GET_IMAGE_INFO & 0xffff,
			       NULL, 0, rsp, ARRAY_SIZE(rsp),
			       MXL371X_MBOX_POLL_BOOT);
	if (ret < 0) {
		dev_info(dev, "Firmware image-info query failed: %d\n", ret);
		return;
	}

	/* The response words hold an ASCII version string (big-endian per word);
	 * lay them out as bytes so socVerStr at offset 0x38 reads correctly. */
	for (i = 0; i < ARRAY_SIZE(rsp); i++)
		*(__be32 *)&rsp[i] = cpu_to_be32(rsp[i]);

	if (ret >= 0x38) {
		char *ver = (char *)rsp + 0x38;
		/* Bound the copy to the bytes the SoC actually returned so a
		 * non-NUL-terminated field can't over-read the stack buffer. */
		size_t avail = min_t(size_t, (size_t)ret, sizeof(rsp)) - 0x38;

		strscpy(priv->fw_version, ver,
			min(sizeof(priv->fw_version), avail + 1));
		dev_info(dev, "MoCA firmware version: %s\n", priv->fw_version);
	}
}

/*
 * Push the MoCA boot configuration and admit the node to the network: send the
 * four parameter payloads, then InitStart.  The payloads are the static
 * byte-exact arrays in mxl371x_payloads.h; only InitParamV2 carries the
 * per-device GUID, patched into a mutable copy.  Sent over the mailbox.
 */
static int mxl371x_send_config(struct phy_device *phydev)
{
	struct device *dev = &phydev->mdio.dev;
	u32 mac_hi, mac_lo;
	u32 rsp[MXL_MOCA_INIT_START_RSP_LEN / 4];
	u32 *v2;
	int ret;

	v2 = kmemdup(init_param_v2_payload, sizeof(init_param_v2_payload),
		     GFP_KERNEL);
	if (!v2)
		return -ENOMEM;

	mxl371x_derive_guid(phydev, &mac_hi, &mac_lo);
	v2[MXL_INITV2_GUID_HI_WORD] = mac_hi;
	v2[MXL_INITV2_GUID_LO_WORD] = mac_lo;

	ret = mxl371x_mbox_cmd(phydev, MXL_MOCA_CMD_INIT_PARAM_V2 & 0xffff,
			       v2, sizeof(init_param_v2_payload), NULL, 0,
			       MXL371X_MBOX_POLL_BOOT);
	kfree(v2);
	if (ret < 0)
		goto fail;

	ret = mxl371x_mbox_cmd(phydev, MXL_MOCA_CMD_RLAPM_V2 & 0xffff,
			       rlapm_v2_payload, sizeof(rlapm_v2_payload),
			       NULL, 0, MXL371X_MBOX_POLL_BOOT);
	if (ret < 0)
		goto fail;

	ret = mxl371x_mbox_cmd(phydev, MXL_MOCA_CMD_SAPM_V2 & 0xffff,
			       sapm_v2_payload, sizeof(sapm_v2_payload),
			       NULL, 0, MXL371X_MBOX_POLL_BOOT);
	if (ret < 0)
		goto fail;

	ret = mxl371x_mbox_cmd(phydev, MXL_MOCA_CMD_INIT_PARAM_V25 & 0xffff,
			       init_param_v25_payload,
			       sizeof(init_param_v25_payload), NULL, 0,
			       MXL371X_MBOX_POLL_BOOT);
	if (ret < 0)
		goto fail;

	dev_info(dev, "MoCA boot config sent; admitting node\n");

	ret = mxl371x_mbox_cmd(phydev, MXL_MOCA_CMD_INIT_START & 0xffff,
			       NULL, 0, rsp, ARRAY_SIZE(rsp),
			       MXL371X_MBOX_POLL_BOOT);
	if (ret < 0)
		goto fail;

	/* rsp word[0] (socGetStatusRsp.status): non-zero = system started. */
	if (ret >= 4 && rsp[0] == 0) {
		dev_warn(dev, "MoCA system start not admitted (status 0)\n");
		return -EAGAIN;
	}
	dev_info(dev, "MoCA system started\n");
	return 0;

fail:
	dev_warn(dev, "MoCA config sequence failed: %d\n", ret);
	return ret;
}

static int mxl371x_load_firmware(struct phy_device *phydev)
{
	struct mxl371x_priv *priv = phydev->priv;
	const struct firmware *fw;
	struct device *dev = &phydev->mdio.dev;
	const struct elf32_hdr *ehdr;
	u32 phoff, phnum, phentsize, i;
	u32 *chunk = NULL;
	int ret;

	/* Check if already loaded */
	if (priv->fw_loaded)
		return 0;

	/* Check if firmware is already running (warm boot) */
	if (mxl371x_check_firmware_running(phydev)) {
		priv->fw_loaded = true;
		dev_info(dev, "Skipping firmware load (already running)\n");
		return 0;
	}

	dev_info(dev, "Loading firmware %s...\n", priv->fw_name);

	ret = request_firmware(&fw, priv->fw_name, dev);
	if (ret) {
		dev_err(dev, "Failed to load firmware: %d\n", ret);
		return ret;
	}

	if (fw->size < sizeof(*ehdr) || fw->size > MXL371X_MAX_FW_SIZE) {
		dev_err(dev, "Invalid firmware size: %zu\n", fw->size);
		ret = -EINVAL;
		goto release_fw;
	}

	/*
	 * The firmware is a 32-bit big-endian ARM ELF image.  Each PT_LOAD
	 * segment must be copied to its physical address (normalised into the
	 * 0x0c400000 load window), with every 32-bit word byte-swapped to SoC
	 * order.  An earlier version wrote the raw file (ELF headers and all) to
	 * address 0, so the CPU found no code at its 0x0c400000 entry point and
	 * never started.
	 */
	ehdr = (const struct elf32_hdr *)fw->data;
	if (memcmp(ehdr->e_ident, ELFMAG, SELFMAG) ||
	    ehdr->e_ident[EI_CLASS] != ELFCLASS32 ||
	    ehdr->e_ident[EI_DATA] != ELFDATA2MSB) {
		dev_err(dev, "Firmware is not a 32-bit big-endian ELF image\n");
		ret = -EINVAL;
		goto release_fw;
	}

	phoff = be32_to_cpu(ehdr->e_phoff);
	phnum = be16_to_cpu(ehdr->e_phnum);
	phentsize = be16_to_cpu(ehdr->e_phentsize);

	if (phentsize < sizeof(struct elf32_phdr) ||
	    phoff + phnum * phentsize > fw->size) {
		dev_err(dev, "Invalid ELF program headers\n");
		ret = -EINVAL;
		goto release_fw;
	}

	chunk = kmalloc(MXL371X_FW_CHUNK_BYTES, GFP_KERNEL);
	if (!chunk) {
		ret = -ENOMEM;
		goto release_fw;
	}

	/*
	 * Hardware reset first: assert the reset line, hold, release.  This
	 * brings the comms-CPU out of its held-in-
	 * reset state so it can run the image we download; without it the SW
	 * reset alone never starts the CPU.
	 *
	 * The reset GPIO is owned by the MDIO core (claimed from the PHY node's
	 * reset-gpios), so we drive it through phy_device_reset() rather than
	 * requesting it ourselves.  Assert (1) then deassert (0) to force a real
	 * pulse; hold/settle timing comes from reset-assert-us/reset-deassert-us
	 * in the DT.
	 */
	dev_info(dev, "SoC hardware reset\n");
	phy_device_reset(phydev, 1);	/* assert */
	msleep(10);
	phy_device_reset(phydev, 0);	/* deassert */
	msleep(50);

	/* Software reset (Leucadia path). */
	ret = mxl371x_write_mem32(phydev, MXL371X_SOC_SWRESET_REG,
				  MXL371X_SOC_SWRESET_VAL);
	if (ret < 0) {
		dev_err(dev, "Failed to reset SoC: %d\n", ret);
		goto free_chunk;
	}
	msleep(50);
	{
		u32 v;

		if (mxl371x_read_mem32(phydev, MXL371X_SOC_SWRESET_DONE_REG,
				       &v) == 0 && v != MXL371X_SOC_SWRESET_DONE_VAL)
			dev_warn(dev, "SW reset unconfirmed (0x%08x != 0x%08x)\n",
				 v, MXL371X_SOC_SWRESET_DONE_VAL);
	}

	dev_info(dev, "Uploading firmware...\n");
	for (i = 0; i < phnum; i++) {
		const struct elf32_phdr *phdr =
			(const struct elf32_phdr *)(fw->data + phoff +
						    i * phentsize);
		u32 type   = be32_to_cpu(phdr->p_type);
		u32 off    = be32_to_cpu(phdr->p_offset);
		u32 filesz = be32_to_cpu(phdr->p_filesz);
		u32 paddr  = be32_to_cpu(phdr->p_paddr);
		u32 dst, done;

		if (type != PT_LOAD || filesz == 0)
			continue;

		if (off > fw->size || filesz > fw->size - off) {
			dev_err(dev, "Segment %u exceeds firmware size\n", i);
			ret = -EINVAL;
			goto free_chunk;
		}

		dst = (paddr & MXL371X_FW_LOAD_MASK) | MXL371X_FW_LOAD_BASE;
		dev_info(dev, "Segment %u: %u bytes -> 0x%08x\n", i, filesz, dst);

		for (done = 0; done < filesz; ) {
			u32 bytes = min_t(u32, MXL371X_FW_CHUNK_BYTES,
					  filesz - done);
			u32 words = (bytes + 3) / 4;
			u32 w;

			for (w = 0; w < words; w++) {
				u32 o = off + done + w * 4;
				u32 b0, b1, b2, b3;

				b0 = fw->data[o];
				b1 = (o + 1 < off + filesz) ? fw->data[o + 1] : 0;
				b2 = (o + 2 < off + filesz) ? fw->data[o + 2] : 0;
				b3 = (o + 3 < off + filesz) ? fw->data[o + 3] : 0;
				/* SoC word order is the big-endian read (ntohl). */
				chunk[w] = (b0 << 24) | (b1 << 16) |
					   (b2 << 8) | b3;
			}

			ret = mxl371x_turbo_write_block(phydev, dst + done,
							chunk, words);
			if (ret < 0) {
				dev_err(dev, "Segment %u upload failed at +%u: %d\n",
					i, done, ret);
				goto free_chunk;
			}

			done += words * 4;
			usleep_range(1000, 2000);
		}
	}

	dev_info(dev, "Firmware upload complete\n");

	/*
	 * Confirm the CPU actually booted using the mailbox handshake: write the
	 * request magic to HOST_MBOX and wait for the running firmware to echo the
	 * ack magic.  FW_STATUS_REG proved unreliable, so it is not trusted here.
	 */
	dev_info(dev, "Booting firmware (mailbox handshake)...\n");
	msleep(200);
	/* Write the request magic, then kick + release the CPU (Leucadia
	 * boot sequence: 0x0c107804=0 then 0x0c107004=0xa). */
	mxl371x_write_mem32(phydev, MXL371X_HOST_MBOX, MXL371X_MBOX_REQ);
	mxl371x_write_mem32(phydev, MXL371X_BOOT_KICK_REG, 0);
	mxl371x_write_mem32(phydev, MXL371X_BOOT_RUN_REG, MXL371X_BOOT_RUN_VAL);

	{
		u32 v, last = MXL371X_MBOX_REQ;

		for (i = 0; i < 2001; i++) {	/* poll up to 0x7d1 times @ 1ms */
			if (mxl371x_read_mem32(phydev, MXL371X_HOST_MBOX, &v) == 0) {
				last = v;
				/*
				 * We wrote the sentinel after download, so the
				 * only thing that can change HOST_MBOX is the SoC
				 * CPU executing -- any change means it booted.
				 * (0/0xffffffff are treated as bus noise.)
				 */
				if (v != MXL371X_MBOX_REQ && v != 0 &&
				    v != 0xffffffff) {
					dev_info(dev, "Firmware started (mailbox 0x%08x)\n",
						 v);
					priv->fw_loaded = true;
					/*
					 * Finalize the mailbox region and do a
					 * first real command round-trip (read
					 * the firmware build string).  Only when
					 * the SoC echoed the genuine ack magic.
					 */
					if ((v == MXL371X_MBOX_ACK ||
					     v == MXL371X_MBOX_ACK2) &&
					    mxl371x_mbox_init(phydev) == 0)
						mxl371x_query_fw_version(phydev);
					ret = 0;
					goto free_chunk;
				}
			}
			usleep_range(1000, 2000);
		}

		dev_err(dev, "Firmware did not start (mailbox stuck at 0x%08x); reset/boot incomplete\n",
			last);
		ret = -ETIMEDOUT;
	}

free_chunk:
	kfree(chunk);
release_fw:
	release_firmware(fw);
	return ret;
}

/* Detect current SGMII/HSGMII configuration from hardware */
static int mxl371x_detect_sgmii_mode(struct phy_device *phydev, u8 *detected_mode)
{
	struct device *dev = &phydev->mdio.dev;
	int ret;
	u16 val;

	/* Read current SGMII configuration from hardware */
	ret = phy_read_paged(phydev, MXL371X_SGMII_CTRL, 0x10);
	if (ret < 0) {
		dev_err(dev, "Failed to read SGMII config: %d\n", ret);
		return ret;
	}

	val = ret & MXL371X_SGMII_MODE_MASK;

	switch (val) {
	case MXL371X_SGMII_MODE_SGMII:
		*detected_mode = MXL371X_SGMII_MODE_SGMII;
		dev_info(dev, "Detected SGMII mode (1000Mbps)\n");
		break;

	case MXL371X_SGMII_MODE_HSGMII:
		*detected_mode = MXL371X_SGMII_MODE_HSGMII;
		dev_info(dev, "Detected HSGMII mode (2500Mbps)\n");
		break;

	case MXL371X_SGMII_MODE_1000BASE_X:
		*detected_mode = MXL371X_SGMII_MODE_1000BASE_X;
		dev_info(dev, "Detected 1000BASE-X mode\n");
		break;

	default:
		dev_warn(dev, "Unknown SGMII mode: 0x%02x\n", val);
		return -EINVAL;
	}

	return 0;
}

static int mxl371x_config_sgmii(struct phy_device *phydev)
{
	struct device *dev = &phydev->mdio.dev;
	int ret;
	u8 mode;
	bool mode_set = false;

	/* 1. Try to use interface mode from device tree */
	switch (phydev->interface) {
	case PHY_INTERFACE_MODE_SGMII:
		mode = MXL371X_SGMII_MODE_SGMII;
		phydev->speed = SPEED_1000;
		mode_set = true;
		dev_info(dev, "Using SGMII mode from device tree (1000Mbps)\n");
		break;

	case PHY_INTERFACE_MODE_2500BASEX:
		mode = MXL371X_SGMII_MODE_HSGMII;
		phydev->speed = SPEED_2500;
		mode_set = true;
		dev_info(dev, "Using HSGMII mode from device tree (2500Mbps)\n");
		break;

	case PHY_INTERFACE_MODE_1000BASEX:
		mode = MXL371X_SGMII_MODE_1000BASE_X;
		phydev->speed = SPEED_1000;
		mode_set = true;
		dev_info(dev, "Using 1000BASE-X mode from device tree\n");
		break;

	default:
		/* 2. Try to detect current hardware configuration (warm boot) */
		ret = mxl371x_detect_sgmii_mode(phydev, &mode);
		if (ret == 0) {
			/* Successfully detected existing mode */
			mode_set = true;
			dev_info(dev, "Using detected hardware configuration\n");

			/* Set speed based on detected mode */
			if (mode == MXL371X_SGMII_MODE_HSGMII)
				phydev->speed = SPEED_2500;
			else
				phydev->speed = SPEED_1000;
		}
		break;
	}

	/* 3. If still not set, make an educated guess */
	if (!mode_set) {
		dev_warn(dev, "No device tree phy-mode and detection failed\n");
		dev_info(dev, "Defaulting to SGMII @ 1000Mbps\n");

		mode = MXL371X_SGMII_MODE_SGMII;
		phydev->speed = SPEED_1000;

		/* Could also check MoCA version as fallback:
		 * if (priv->moca_version >= MOCA_VER_2_5) {
		 * mode = MXL371X_SGMII_MODE_HSGMII;
		 * phydev->speed = SPEED_2500;
		 * }
		 */
	}

	/* Configure the SGMII interface */
	ret = phy_modify_paged(phydev, MXL371X_SGMII_CTRL, 0x10,
			       MXL371X_SGMII_MODE_MASK, mode);
	if (ret < 0) {
		dev_err(dev, "Failed to configure SGMII mode: %d\n", ret);
		return ret;
	}

	phydev->duplex = DUPLEX_FULL;
	dev_info(dev, "Configured %s @ %dMbps\n",
		 mode == MXL371X_SGMII_MODE_HSGMII ? "HSGMII" :
		 mode == MXL371X_SGMII_MODE_1000BASE_X ? "1000BASE-X" : "SGMII",
		 phydev->speed);

	return 0;
}

static int mxl371x_config_init(struct phy_device *phydev)
{
	struct mxl371x_priv *priv = phydev->priv;
	struct device *dev = &phydev->mdio.dev;
	int ret;
	bool warm_boot = false;

	/* Get device information */
	ret = mxl371x_get_device_info(phydev);
	if (ret < 0)
		return ret;

	/* Check if this is a warm boot */
	warm_boot = mxl371x_check_firmware_running(phydev);
	if (warm_boot) {
		priv->fw_loaded = true;
		dev_info(dev, "Warm boot detected, skipping firmware load\n");
	}

	/*
	 * Load firmware (skipped if already running).  A failure here is not
	 * fatal: the SGMII/HSGMII backhaul link can still come up without the
	 * MoCA management firmware, so log honestly and continue rather than
	 * failing the PHY probe.  priv->fw_loaded reflects the real state.
	 */
	ret = mxl371x_load_firmware(phydev);
	if (ret < 0)
		dev_warn(dev, "Firmware not confirmed running (%d); MoCA management unavailable, link may still work\n",
			 ret);

	/*
	 * Push the boot configuration (band plan, power maps, GUID) and admit
	 * the node to the MoCA network over the mailbox.  Without this the
	 * firmware boots unconfigured and the coax link stays down.  A failure
	 * is not fatal to the PHY probe -- the SGMII backhaul still comes up.
	 */
	if (priv->fw_loaded && priv->mbox_ready) {
		if (mxl371x_send_config(phydev) < 0)
			dev_warn(dev, "MoCA boot config not applied; link may stay down\n");

		mxl371x_read_moca_status(phydev);
	}

	/* Configure SGMII/HSGMII interface
	 * Uses device tree phy-mode if set, otherwise detects from hardware */
	ret = mxl371x_config_sgmii(phydev);
	if (ret < 0) {
		dev_err(dev, "SGMII configuration failed: %d\n", ret);
		return ret;
	}

	/* Create sysfs attributes */
	ret = sysfs_create_group(&phydev->mdio.dev.kobj, &mxl371x_attr_group);
	if (ret < 0) {
		dev_err(dev, "Failed to create sysfs attributes: %d\n", ret);
		return ret;
	}

	/* Initialize hwmon temperature sensor */
	ret = mxl371x_hwmon_init(phydev);
	if (ret < 0)
		dev_warn(dev, "Failed to init hwmon: %d\n", ret);

	/* Start statistics polling */
	schedule_delayed_work(&priv->stats_poll, HZ);

	if (warm_boot) {
		dev_info(dev, "MoCA PHY initialized (warm boot, MoCA v%u.%u, %uMbps)\n",
			 priv->moca_version >> 4, priv->moca_version & 0xf,
			 phydev->speed);
	} else {
		dev_info(dev, "MoCA PHY initialized (cold boot, MoCA v%u.%u, %uMbps)\n",
			 priv->moca_version >> 4, priv->moca_version & 0xf,
			 phydev->speed);
	}

	return 0;
}

static int mxl371x_probe(struct phy_device *phydev)
{
	struct mxl371x_priv *priv;

	priv = devm_kzalloc(&phydev->mdio.dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	phydev->priv = priv;
	priv->phydev = phydev;
	mutex_init(&priv->mbox_lock);
	INIT_DELAYED_WORK(&priv->stats_poll, mxl371x_stats_poll_work);
	return 0;
}

static void mxl371x_remove(struct phy_device *phydev)
{
	struct mxl371x_priv *priv = phydev->priv;

	cancel_delayed_work_sync(&priv->stats_poll);
	sysfs_remove_group(&phydev->mdio.dev.kobj, &mxl371x_attr_group);
}

static int mxl371x_read_status(struct phy_device *phydev)
{
	struct mxl371x_priv *priv = phydev->priv;
	int ret;

	ret = genphy_read_status(phydev);
	if (ret < 0)
		return ret;

	/*
	 * Report the MoCA link from the value cached by the stats workqueue --
	 * do not issue mailbox commands here, as read_status runs under the phy
	 * lock and a mailbox round-trip can block for seconds on a wedged SoC.
	 */
	if (priv->fw_loaded)
		phydev->link = (priv->link_status == MOCA_LINK_UP);

	return 0;
}

static int mxl371x_config_aneg(struct phy_device *phydev)
{
	phydev->autoneg = AUTONEG_DISABLE;
	phydev->duplex = DUPLEX_FULL;
	return 0;
}

static int mxl371x_suspend(struct phy_device *phydev)
{
	struct mxl371x_priv *priv = phydev->priv;

	cancel_delayed_work_sync(&priv->stats_poll);
	return genphy_suspend(phydev);
}

static int mxl371x_resume(struct phy_device *phydev)
{
	struct mxl371x_priv *priv = phydev->priv;

	/*
	 * A PHY suspend/resume does NOT reset the MoCA SoC -- the firmware keeps
	 * running across it -- so fw_loaded must NOT be cleared here.  Clearing
	 * it (and never reloading, since config_init does not re-run on resume)
	 * silently wedged the status poll after the first phylink resume.
	 */
	schedule_delayed_work(&priv->stats_poll, HZ);
	return genphy_resume(phydev);
}

static int mxl371x_get_features(struct phy_device *phydev)
{
	/* MoCA is a fixed-speed backplane-style interface; standard copper
	 * autoneg registers are not implemented.  Declare exactly what this
	 * chip supports so phylink validation passes. */
	linkmode_zero(phydev->supported);
	linkmode_set_bit(ETHTOOL_LINK_MODE_2500baseX_Full_BIT, phydev->supported);
	linkmode_set_bit(ETHTOOL_LINK_MODE_1000baseX_Full_BIT, phydev->supported);
	linkmode_set_bit(ETHTOOL_LINK_MODE_FIBRE_BIT, phydev->supported);
	linkmode_set_bit(ETHTOOL_LINK_MODE_Pause_BIT, phydev->supported);
	linkmode_set_bit(ETHTOOL_LINK_MODE_Asym_Pause_BIT, phydev->supported);
	return 0;
}

static int mxl371x_match_phy_device(struct phy_device *phydev,
				    const struct phy_driver *drv)
{
	return ((phydev->phy_id & MXL371X_OUI_MASK) == MXL371X_OUI);
}

static struct phy_driver mxl371x_drivers[] = {
	{
		PHY_ID_MATCH_EXACT(MXL3710_PHY_ID),
		.name		= "MaxLinear MXL3710 MoCA 2.5",
		.probe		= mxl371x_probe,
		.remove		= mxl371x_remove,
		.get_features	= mxl371x_get_features,
		.config_init	= mxl371x_config_init,
		.config_aneg	= mxl371x_config_aneg,
		.read_status	= mxl371x_read_status,
		.get_phy_stats	= mxl371x_get_phy_stats,
		.suspend	= mxl371x_suspend,
		.resume		= mxl371x_resume,
		.read_page	= mxl371x_read_page,
		.write_page	= mxl371x_write_page,
	}, {
		PHY_ID_MATCH_EXACT(MXL3711_PHY_ID),
		.name		= "MaxLinear MXL3711 MoCA 2.5",
		.probe		= mxl371x_probe,
		.remove		= mxl371x_remove,
		.get_features	= mxl371x_get_features,
		.config_init	= mxl371x_config_init,
		.config_aneg	= mxl371x_config_aneg,
		.read_status	= mxl371x_read_status,
		.get_phy_stats	= mxl371x_get_phy_stats,
		.suspend	= mxl371x_suspend,
		.resume		= mxl371x_resume,
		.read_page	= mxl371x_read_page,
		.write_page	= mxl371x_write_page,
	}, {
		.match_phy_device = mxl371x_match_phy_device,
		.name		= "MaxLinear MXL371x MoCA 2.5",
		.probe		= mxl371x_probe,
		.remove		= mxl371x_remove,
		.get_features	= mxl371x_get_features,
		.config_init	= mxl371x_config_init,
		.config_aneg	= mxl371x_config_aneg,
		.read_status	= mxl371x_read_status,
		.get_phy_stats	= mxl371x_get_phy_stats,
		.suspend	= mxl371x_suspend,
		.resume		= mxl371x_resume,
		.read_page	= mxl371x_read_page,
		.write_page	= mxl371x_write_page,
	},
};

module_phy_driver(mxl371x_drivers);

static const struct mdio_device_id __maybe_unused mxl371x_tbl[] = {
	{ PHY_ID_MATCH_VENDOR(MXL371X_OUI) },
	{ PHY_ID_MATCH_EXACT(MXL3710_PHY_ID) },
	{ PHY_ID_MATCH_EXACT(MXL3711_PHY_ID) },
	{ }
};

MODULE_DEVICE_TABLE(mdio, mxl371x_tbl);
MODULE_FIRMWARE(MXL371X_FW_LEUCADIA);
MODULE_FIRMWARE(MXL371X_FW_CARDIFF);
MODULE_DESCRIPTION("MaxLinear MXL371x MoCA 2.5 PHY driver");
MODULE_AUTHOR("Kenneth Kasilag");
MODULE_LICENSE("GPL");
