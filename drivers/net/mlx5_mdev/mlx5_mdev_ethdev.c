/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2015 6WIND S.A.
 * Copyright 2015 Mellanox.
 */

#define _GNU_SOURCE

#include <stddef.h>
#include <assert.h>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <dirent.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/utsname.h>
#include <netinet/in.h>
#include <linux/ethtool.h>
#include <linux/sockios.h>
#include <linux/version.h>
#include <fcntl.h>
#include <stdalign.h>
#include <sys/un.h>

#include <rte_atomic.h>
#include <rte_ethdev_driver.h>
#include <rte_bus_pci.h>
#include <rte_mbuf.h>
#include <rte_common.h>
#include <rte_interrupts.h>
#include <rte_alarm.h>
#include <rte_malloc.h>

#include "mlx5_mdev_priv.h"
#include "mlx5_mdev_utils.h"

/* Add defines in case the running kernel is not the same as user headers. */
#ifndef ETHTOOL_GLINKSETTINGS
struct ethtool_link_settings {
	uint32_t cmd;
	uint32_t speed;
	uint8_t duplex;
	uint8_t port;
	uint8_t phy_address;
	uint8_t autoneg;
	uint8_t mdio_support;
	uint8_t eth_to_mdix;
	uint8_t eth_tp_mdix_ctrl;
	int8_t link_mode_masks_nwords;
	uint32_t reserved[8];
	uint32_t link_mode_masks[];
};

#define ETHTOOL_GLINKSETTINGS 0x0000004c
#define ETHTOOL_LINK_MODE_1000baseT_Full_BIT 5
#define ETHTOOL_LINK_MODE_Autoneg_BIT 6
#define ETHTOOL_LINK_MODE_1000baseKX_Full_BIT 17
#define ETHTOOL_LINK_MODE_10000baseKX4_Full_BIT 18
#define ETHTOOL_LINK_MODE_10000baseKR_Full_BIT 19
#define ETHTOOL_LINK_MODE_10000baseR_FEC_BIT 20
#define ETHTOOL_LINK_MODE_20000baseMLD2_Full_BIT 21
#define ETHTOOL_LINK_MODE_20000baseKR2_Full_BIT 22
#define ETHTOOL_LINK_MODE_40000baseKR4_Full_BIT 23
#define ETHTOOL_LINK_MODE_40000baseCR4_Full_BIT 24
#define ETHTOOL_LINK_MODE_40000baseSR4_Full_BIT 25
#define ETHTOOL_LINK_MODE_40000baseLR4_Full_BIT 26
#define ETHTOOL_LINK_MODE_56000baseKR4_Full_BIT 27
#define ETHTOOL_LINK_MODE_56000baseCR4_Full_BIT 28
#define ETHTOOL_LINK_MODE_56000baseSR4_Full_BIT 29
#define ETHTOOL_LINK_MODE_56000baseLR4_Full_BIT 30
#endif
#ifndef HAVE_ETHTOOL_LINK_MODE_25G
#define ETHTOOL_LINK_MODE_25000baseCR_Full_BIT 31
#define ETHTOOL_LINK_MODE_25000baseKR_Full_BIT 32
#define ETHTOOL_LINK_MODE_25000baseSR_Full_BIT 33
#endif
#ifndef HAVE_ETHTOOL_LINK_MODE_50G
#define ETHTOOL_LINK_MODE_50000baseCR2_Full_BIT 34
#define ETHTOOL_LINK_MODE_50000baseKR2_Full_BIT 35
#endif
#ifndef HAVE_ETHTOOL_LINK_MODE_100G
#define ETHTOOL_LINK_MODE_100000baseKR4_Full_BIT 36
#define ETHTOOL_LINK_MODE_100000baseSR4_Full_BIT 37
#define ETHTOOL_LINK_MODE_100000baseCR4_Full_BIT 38
#define ETHTOOL_LINK_MODE_100000baseLR4_ER4_Full_BIT 39
#endif

/**
 * Check if the counter is located on ib counters file.
 *
 * @param[in] cntr
 *   Counter name.
 *
 * @return
 *   1 if counter is located on ib counters file , 0 otherwise.
 */
int
priv_is_ib_cntr(const char *cntr)
{
	if (!strcmp(cntr, "out_of_buffer"))
		return 1;
	return 0;
}

/**
 * Get interface name from private structure.
 *
 * @param[in] priv
 *   Pointer to private structure.
 * @param[out] ifname
 *   Interface name output buffer.
 *
 * @return
 *   0 on success, -1 on failure and errno is set.
 */
int
priv_get_ifname(const struct mlx5_mdev_priv *priv, char (*ifname)[IFNAMSIZ])
{
	DIR *dir;
	struct dirent *dent;
	unsigned int dev_type = 0;
	unsigned int dev_port_prev = ~0u;
	char match[256] = "";

	{
		MKSTR(path, "%s/device/net", priv->ibdev_path);

		dir = opendir(path);
		if (dir == NULL)
			return -1;
	}
	while ((dent = readdir(dir)) != NULL) {
		char *name = dent->d_name;
		FILE *file;
		unsigned int dev_port;
		int r;

		if ((name[0] == '.') &&
		    ((name[1] == '\0') ||
		     ((name[1] == '.') && (name[2] == '\0'))))
			continue;

		MKSTR(path, "%s/device/net/%s/%s",
		      priv->ibdev_path, name,
		      (dev_type ? "dev_id" : "dev_port"));

		file = fopen(path, "rb");
		if (file == NULL) {
			if (errno != ENOENT)
				continue;
			/*
			 * Switch to dev_id when dev_port does not exist as
			 * is the case with Linux kernel versions < 3.15.
			 */
try_dev_id:
			match[0] = '\0';
			if (dev_type)
				break;
			dev_type = 1;
			dev_port_prev = ~0u;
			rewinddir(dir);
			continue;
		}
		r = fscanf(file, (dev_type ? "%x" : "%u"), &dev_port);
		fclose(file);
		if (r != 1)
			continue;
		/*
		 * Switch to dev_id when dev_port returns the same value for
		 * all ports. May happen when using a MOFED release older than
		 * 3.0 with a Linux kernel >= 3.15.
		 */
		if (dev_port == dev_port_prev)
			goto try_dev_id;
		dev_port_prev = dev_port;
		if (dev_port == (priv->port - 1u))
			snprintf(match, sizeof(match), "%s", name);
	}
	closedir(dir);
	if (match[0] == '\0')
		return -1;
	strncpy(*ifname, match, sizeof(*ifname));
	return 0;
}

/**
 * Read from sysfs entry.
 *
 * @param[in] priv
 *   Pointer to private structure.
 * @param[in] entry
 *   Entry name relative to sysfs path.
 * @param[out] buf
 *   Data output buffer.
 * @param size
 *   Buffer size.
 *
 * @return
 *   0 on success, -1 on failure and errno is set.
 */
static int
priv_sysfs_read(const struct mlx5_mdev_priv *priv, const char *entry,
		char *buf, size_t size)
{
	char ifname[IFNAMSIZ];
	FILE *file;
	int ret;
	int err;

	if (priv_get_ifname(priv, &ifname))
		return -1;

	if (priv_is_ib_cntr(entry)) {
		MKSTR(path, "%s/ports/1/hw_counters/%s",
		      priv->ibdev_path, entry);
		file = fopen(path, "rb");
	} else {
		MKSTR(path, "%s/device/net/%s/%s",
		      priv->ibdev_path, ifname, entry);
		file = fopen(path, "rb");
	}
	if (file == NULL)
		return -1;
	ret = fread(buf, 1, size, file);
	err = errno;
	if (((size_t)ret < size) && (ferror(file)))
		ret = -1;
	else
		ret = size;
	fclose(file);
	errno = err;
	return ret;
}

/**
 * Write to sysfs entry.
 *
 * @param[in] priv
 *   Pointer to private structure.
 * @param[in] entry
 *   Entry name relative to sysfs path.
 * @param[in] buf
 *   Data buffer.
 * @param size
 *   Buffer size.
 *
 * @return
 *   0 on success, -1 on failure and errno is set.
 */
static int
priv_sysfs_write(const struct mlx5_mdev_priv *priv, const char *entry,
		 char *buf, size_t size)
{
	char ifname[IFNAMSIZ];
	FILE *file;
	int ret;
	int err;

	if (priv_get_ifname(priv, &ifname))
		return -1;

	MKSTR(path, "%s/device/net/%s/%s", priv->ibdev_path, ifname, entry);

	file = fopen(path, "wb");
	if (file == NULL)
		return -1;
	ret = fwrite(buf, 1, size, file);
	err = errno;
	if (((size_t)ret < size) || (ferror(file)))
		ret = -1;
	else
		ret = size;
	fclose(file);
	errno = err;
	return ret;
}

/**
 * Get unsigned long sysfs property.
 *
 * @param priv
 *   Pointer to private structure.
 * @param[in] name
 *   Entry name relative to sysfs path.
 * @param[out] value
 *   Value output buffer.
 *
 * @return
 *   0 on success, -1 on failure and errno is set.
 */
static int
priv_get_sysfs_ulong(struct mlx5_mdev_priv *priv, const char *name, unsigned long *value)
{
	int ret;
	unsigned long value_ret;
	char value_str[32];

	ret = priv_sysfs_read(priv, name, value_str, (sizeof(value_str) - 1));
	if (ret == -1) {
		DEBUG("cannot read %s value from sysfs: %s",
		      name, strerror(errno));
		return -1;
	}
	value_str[ret] = '\0';
	errno = 0;
	value_ret = strtoul(value_str, NULL, 0);
	if (errno) {
		DEBUG("invalid %s value `%s': %s", name, value_str,
		      strerror(errno));
		return -1;
	}
	*value = value_ret;
	return 0;
}

/**
 * Set unsigned long sysfs property.
 *
 * @param priv
 *   Pointer to private structure.
 * @param[in] name
 *   Entry name relative to sysfs path.
 * @param value
 *   Value to set.
 *
 * @return
 *   0 on success, -1 on failure and errno is set.
 */
static int
priv_set_sysfs_ulong(struct mlx5_mdev_priv *priv, const char *name, unsigned long value)
{
	int ret;
	MKSTR(value_str, "%lu", value);

	ret = priv_sysfs_write(priv, name, value_str, (sizeof(value_str) - 1));
	if (ret == -1) {
		DEBUG("cannot write %s `%s' (%lu) to sysfs: %s",
		      name, value_str, value, strerror(errno));
		return -1;
	}
	return 0;
}


/**
 * Return the number of active VFs for the current device.
 *
 * @param[in] priv
 *   Pointer to private structure.
 * @param[out] num_vfs
 *   Number of active VFs.
 *
 * @return
 *   0 on success, -1 on failure and errno is set.
 */
int
priv_get_num_vfs(struct mlx5_mdev_priv *priv, uint16_t *num_vfs)
{
	/* The sysfs entry name depends on the operating system. */
	const char **name = (const char *[]){
		"device/sriov_numvfs",
		"device/mlx5_num_vfs",
		NULL,
	};
	int ret;

	do {
		unsigned long ulong_num_vfs;

		ret = priv_get_sysfs_ulong(priv, *name, &ulong_num_vfs);
		if (!ret)
			*num_vfs = ulong_num_vfs;
	} while (*(++name) && ret);
	return ret;
}

/**
 * Get device MTU.
 *
 * @param priv
 *   Pointer to private structure.
 * @param[out] mtu
 *   MTU value output buffer.
 *
 * @return
 *   0 on success, -1 on failure and errno is set.
 */
int
priv_get_mtu(struct mlx5_mdev_priv *priv, uint16_t *mtu)
{
	unsigned long ulong_mtu;

	if (priv_get_sysfs_ulong(priv, "mtu", &ulong_mtu) == -1)
		return -1;
	*mtu = ulong_mtu;
	return 0;
}

/**
 * Set device flags.
 *
 * @param priv
 *   Pointer to private structure.
 * @param keep
 *   Bitmask for flags that must remain untouched.
 * @param flags
 *   Bitmask for flags to modify.
 *
 * @return
 *   0 on success, -1 on failure and errno is set.
 */
int
priv_set_flags(struct mlx5_mdev_priv *priv, unsigned int keep, unsigned int flags)
{
	unsigned long tmp;

	if (priv_get_sysfs_ulong(priv, "flags", &tmp) == -1)
		return -1;
	tmp &= keep;
	tmp |= (flags & (~keep));
	return priv_set_sysfs_ulong(priv, "flags", tmp);
}

/**
 * Perform ifreq ioctl() on associated Ethernet device.
 *
 * @param[in] priv
 *   Pointer to private structure.
 * @param req
 *   Request number to pass to ioctl().
 * @param[out] ifr
 *   Interface request structure output buffer.
 *
 * @return
 *   0 on success, -1 on failure and errno is set.
 */
int
priv_ifreq(const struct mlx5_mdev_priv *priv, int req, struct ifreq *ifr)
{
	int sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_IP);
	int ret = -1;

	if (sock == -1)
		return ret;
	if (priv_get_ifname(priv, &ifr->ifr_name) == 0)
		ret = ioctl(sock, req, ifr);
	close(sock);
	return ret;
}

/**
 * Get PCI information from struct ibv_device.
 *
 * @param device
 *   Pointer to Ethernet device structure.
 * @param[out] pci_addr
 *   PCI bus address output buffer.
 *
 * @return
 *   0 on success, -1 on failure and errno is set.
 */
int
mlx5_mdev_device_to_pci_addr(const struct devx_device *device,
			     struct rte_pci_addr *pci_addr)
{
	FILE *file;
	char line[32];
	MKSTR(path, "%s/device/uevent", device->ibdev_path);

	file = fopen(path, "rb");
	if (file == NULL)
		return -1;
	while (fgets(line, sizeof(line), file) == line) {
		size_t len = strlen(line);
		int ret;

		/* Truncate long lines. */
		if (len == (sizeof(line) - 1))
			while (line[(len - 1)] != '\n') {
				ret = fgetc(file);
				if (ret == EOF)
					break;
				line[(len - 1)] = ret;
			}
		/* Extract information. */
		if (sscanf(line,
			   "PCI_SLOT_NAME="
			   "%" SCNx32 ":%" SCNx8 ":%" SCNx8 ".%" SCNx8 "\n",
			   &pci_addr->domain,
			   &pci_addr->bus,
			   &pci_addr->devid,
			   &pci_addr->function) == 4) {
			ret = 0;
			break;
		}
	}
	fclose(file);
	return 0;
}

/**
 * Ethernet device configuration.
 *
 * Prepare the driver for a given number of TX and RX queues.
 *
 * @param dev
 *   Pointer to Ethernet device structure.
 *
 * @return
 *   0 on success, errno value on failure.
 */
static int
dev_configure(struct rte_eth_dev *dev)
{
	struct mlx5_mdev_priv *priv = dev->data->dev_private;
//	unsigned int rxqs_n = dev->data->nb_rx_queues;
	unsigned int txqs_n = dev->data->nb_tx_queues;
//	unsigned int i;
//	unsigned int j;
//	unsigned int reta_idx_n;
//	const uint8_t use_app_rss_key =
//		!!dev->data->dev_conf.rx_adv_conf.rss_conf.rss_key;
	printf("oooOri start of configure function\n");
	uint64_t supp_tx_offloads = mlx5_priv_get_tx_port_offloads(priv);
	uint64_t tx_offloads = dev->data->dev_conf.txmode.offloads;
	if ((tx_offloads & supp_tx_offloads) != tx_offloads) {
			ERROR("Some Tx offloads are not supported "
			      "requested 0x%" PRIx64 " supported 0x%" PRIx64,
			      tx_offloads, supp_tx_offloads);
			// return ENOTSUP;   TODO: Return me
	}
	priv->txqs = (void *)dev->data->tx_queues;
	if (txqs_n != priv->txqs_n) {
		INFO("%p: TX queues number update: %u -> %u",
		     (void *)dev, priv->txqs_n, txqs_n);
		priv->txqs_n = txqs_n;
	}
	printf("oooOri end of configure function\n");
#if 0
	uint64_t supp_rx_offloads =
		(mlx5_priv_get_rx_port_offloads(priv) |
		 mlx5_priv_get_rx_queue_offloads(priv));
	uint64_t rx_offloads = dev->data->dev_conf.rxmode.offloads;


	if ((rx_offloads & supp_rx_offloads) != rx_offloads) {
		ERROR("Some Rx offloads are not supported "
		      "requested 0x%" PRIx64 " supported 0x%" PRIx64,
		      rx_offloads, supp_rx_offloads);
		// return ENOTSUP; TODO: Return me
	}

	if (use_app_rss_key &&
	    (dev->data->dev_conf.rx_adv_conf.rss_conf.rss_key_len !=
	     rss_hash_default_key_len)) {
		/* MLX5 RSS only support 40bytes key. */
		ERROR("===== Invalid RSS key");	// TODO: Remove me
		// return EINVAL; TODO: Return me
	}
	priv->rss_conf.rss_key =
		rte_realloc(priv->rss_conf.rss_key,
			    rss_hash_default_key_len, 0);
	if (!priv->rss_conf.rss_key) {
		ERROR("cannot allocate RSS hash key memory (%u)", rxqs_n);
		return ENOMEM;
	}
	memcpy(priv->rss_conf.rss_key,
	       use_app_rss_key ?
	       dev->data->dev_conf.rx_adv_conf.rss_conf.rss_key :
	       rss_hash_default_key,
	       rss_hash_default_key_len);
	priv->rss_conf.rss_key_len = rss_hash_default_key_len;
	priv->rss_conf.rss_hf = dev->data->dev_conf.rx_adv_conf.rss_conf.rss_hf;
	priv->rxqs = (void *)dev->data->rx_queues;

	if (rxqs_n > priv->config.ind_table_max_size) {
		ERROR("cannot handle this many RX queues (%u)", rxqs_n);
		return EINVAL;
	}
	if (rxqs_n == priv->rxqs_n)
		return 0;
	INFO("%p: RX queues number update: %u -> %u",
	     (void *)dev, priv->rxqs_n, rxqs_n);
	priv->rxqs_n = rxqs_n;
	ERROR("=========== %s: %d", __FUNCTION__, __LINE__);
	/* If the requested number of RX queues is not a power of two, use the
	 * maximum indirection table size for better balancing.
	 * The result is always rounded to the next power of two. */
	reta_idx_n = (1 << log2above((rxqs_n & (rxqs_n - 1)) ?
				     priv->config.ind_table_max_size :
				     rxqs_n));
	if (priv_rss_reta_index_resize(priv, reta_idx_n))
		return ENOMEM;
	ERROR("=========== %s: %d", __FUNCTION__, __LINE__);
	/* When the number of RX queues is not a power of two, the remaining
	 * table entries are padded with reused WQs and hashes are not spread
	 * uniformly. */
	for (i = 0, j = 0; (i != reta_idx_n); ++i) {
		(*priv->reta_idx)[i] = j;
		if (++j == rxqs_n)
			j = 0;
	}
#endif
	return 0;
}

/**
 * DPDK callback for Ethernet device configuration.
 *
 * @param dev
 *   Pointer to Ethernet device structure.
 *
 * @return
 *   0 on success, negative errno value on failure.
 */
int
mlx5_dev_configure(struct rte_eth_dev *dev)
{
	struct mlx5_mdev_priv *priv = dev->data->dev_private;
	int ret;

	priv_lock(priv);
	ret = dev_configure(dev);
	assert(ret >= 0);
	priv_unlock(priv);
	return -ret;
}

/**
 * Configure the TX function to use.
 *
 * @param priv
 *   Pointer to private data structure.
 * @param dev
 *   Pointer to rte_eth_dev structure.
 *
 * @return
 *   Pointer to selected Tx burst function.
 */
eth_tx_burst_t
priv_select_tx_function(struct mlx5_mdev_priv __rte_unused *priv, struct rte_eth_dev __rte_unused *dev)
{
#if 0
	eth_tx_burst_t tx_pkt_burst = mlx5_tx_burst;
	struct mlx5_mdev_dev_config *config = &priv->config;
	uint64_t tx_offloads = dev->data->dev_conf.txmode.offloads;
	int tso = !!(tx_offloads & (DEV_TX_OFFLOAD_TCP_TSO |
				    DEV_TX_OFFLOAD_VXLAN_TNL_TSO |
				    DEV_TX_OFFLOAD_GRE_TNL_TSO));
	int vlan_insert = !!(tx_offloads & DEV_TX_OFFLOAD_VLAN_INSERT);

	assert(priv != NULL);
	/* Select appropriate TX function. */
	if (vlan_insert || tso)
		return tx_pkt_burst;
	if (config->mps == MLX5_MPW_ENHANCED) {
		if (priv_check_vec_tx_support(priv, dev) > 0) {
			if (priv_check_raw_vec_tx_support(priv, dev) > 0)
				tx_pkt_burst = mlx5_tx_burst_raw_vec;
			else
				tx_pkt_burst = mlx5_tx_burst_vec;
			DEBUG("selected Enhanced MPW TX vectorized function");
		} else {
			tx_pkt_burst = mlx5_tx_burst_empw;
			DEBUG("selected Enhanced MPW TX function");
		}
	} else if (config->mps && (config->txq_inline > 0)) {
		tx_pkt_burst = mlx5_tx_burst_mpw_inline;
		DEBUG("selected MPW inline TX function");
	} else if (config->mps) {
		tx_pkt_burst = mlx5_tx_burst_mpw;
		DEBUG("selected MPW TX function");
	}
#endif
	return NULL;
}
