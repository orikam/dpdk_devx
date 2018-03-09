/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2015 Mellanox.
 */
#include <sys/ioctl.h>
#include <net/if.h> 
#include <unistd.h>
#include <netinet/in.h>
#include <string.h>
	#include <stddef.h>
	#include <unistd.h>
	#include <string.h>
	#include <stdint.h>
	#include <stdlib.h>
	#include <errno.h>
	#include <net/if.h>
	#include <assert.h>
	#include <sys/mman.h>

	#include <rte_io.h>
	#include <rte_pci.h>
	#include <rte_log.h>
	#include <rte_malloc.h>
	#include <rte_memzone.h>
	#include <rte_eal_memconfig.h>
	#include <rte_kvargs.h>

	#include <rte_ether.h>
	#include <rte_ethdev.h>
	#include <rte_ethdev_pci.h>
	#include <rte_eth_ctrl.h>

	#include "devx.h"
	#include "mlx5_mdev_utils.h"
	//#include "mdev_lib.h"
	#include "mdev_prm.h"
	#include "mlx5_mdev_priv.h"

	/* Device parameter to enable RX completion queue compression. */
	#define MLX5_RXQ_CQE_COMP_EN "rxq_cqe_comp_en"

	/* Device parameter to configure inline send. */
	#define MLX5_TXQ_INLINE "txq_inline"

	/*
	 * Device parameter to configure the number of TX queues threshold for
	 * enabling inline send.
	 */
	#define MLX5_TXQS_MIN_INLINE "txqs_min_inline"

	/* Device parameter to enable multi-packet send WQEs. */
	#define MLX5_TXQ_MPW_EN "txq_mpw_en"

	/* Device parameter to include 2 dsegs in the title WQEBB. */
	#define MLX5_TXQ_MPW_HDR_DSEG_EN "txq_mpw_hdr_dseg_en"

	/* Device parameter to limit the size of inlining packet. */
	#define MLX5_TXQ_MAX_INLINE_LEN "txq_max_inline_len"

	/* Device parameter to enable hardware Tx vector. */
	#define MLX5_TX_VEC_EN "tx_vec_en"

	/* Device parameter to enable hardware Rx vector. */
	#define MLX5_RX_VEC_EN "rx_vec_en"

	static inline struct mlx5_mdev_memzone * alloc_pinned(void * edev,
						const char *name,
						size_t size,
						size_t align)
	{
		const struct rte_memzone *rte_mz = rte_eth_dma_zone_reserve(edev,
					name, 0, size, align,
					((struct rte_eth_dev *)edev)->data->numa_node);
		struct mlx5_mdev_memzone *mz = rte_zmalloc("mdev",
						sizeof(struct mlx5_mdev_memzone), 64);

		if(!mz)
			return mz;
		mz->addr = rte_mz->addr;
		mz->phys_addr = rte_mz->iova;
		return mz;
	}
	#if 0
	static int get_mac_address(uint8_t *mac_address)
	{
		struct ifreq ifr;
		struct ifconf ifc;
		char buf[1024];
		int success = 0;
		struct ifreq* it;
		const struct ifreq* end;
		int i;
	 
		int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
		if (sock == -1) { /* handle error*/ };

		ifc.ifc_len = sizeof(buf);
		ifc.ifc_buf = buf;
		if (ioctl(sock, SIOCGIFCONF, &ifc) == -1) { /* handle error */ }

		it = ifc.ifc_req;
		end = it + (ifc.ifc_len / sizeof(struct ifreq));

		for (; it != end; ++it) {
			strcpy(ifr.ifr_name, it->ifr_name);
			if (ioctl(sock, SIOCGIFFLAGS, &ifr) == 0) {
				if (! (ifr.ifr_flags & IFF_LOOPBACK)) { // don't count loopback
					if (ioctl(sock, SIOCGIFHWADDR, &ifr) == 0) {
					success = 0;
					break;
					}
				}
			}
			else
				success = 1;
		}
		if (!success) memcpy(mac_address, ifr.ifr_hwaddr.sa_data, 6);
		printf("mac address: ");
		for (i=0; i<6; i++) {
			printf("%x,",mac_address[i]);
		}
		printf("\n");
		return success;
	}
	#endif


	static void mlx5_mdev_infos_get(struct rte_eth_dev *edev, // TODO: review field-by-field, considering dev caps
					struct rte_eth_dev_info *info)
	{
		info->pci_dev		 = RTE_ETH_DEV_TO_PCI(edev);
		info->min_rx_bufsize	 = 32;
		info->max_rx_pktlen	 = 65536;
		info->max_rx_queues	 = 1;
		info->max_tx_queues	 = 1;
		info->max_mac_addrs	 = 1;
		info->rx_offload_capa	 = DEV_RX_OFFLOAD_IPV4_CKSUM;
		info->rx_offload_capa	|= DEV_RX_OFFLOAD_UDP_CKSUM;
		info->rx_offload_capa	|= DEV_RX_OFFLOAD_TCP_CKSUM;
		info->rx_offload_capa	|= DEV_RX_OFFLOAD_VLAN_STRIP;
		info->tx_offload_capa	 = DEV_TX_OFFLOAD_IPV4_CKSUM;
		info->tx_offload_capa	|= DEV_TX_OFFLOAD_UDP_CKSUM;
		info->tx_offload_capa	|= DEV_TX_OFFLOAD_TCP_CKSUM;
		info->speed_capa	 = ETH_LINK_SPEED_10G;
	}

	static const struct eth_dev_ops mlx5_mdev_dev_ops = { // TODO...
		.dev_start		= mlx5_mdev_start,
		.dev_infos_get		= mlx5_mdev_infos_get,
		.tx_queue_setup = mlx5_tx_queue_setup,
		.dev_configure = mlx5_dev_configure,
	//	.dev_stop = mlx5_dev_stop,
		.dev_set_link_down = mlx5_mdev_set_link_down,
		.dev_set_link_up = mlx5_mdev_set_link_up,
		.link_update = mlx5_mdev_link_update,
		.stats_get = mlx5_mdev_stats_get,
		.stats_reset = mlx5_mdev_stats_reset,


	//	.dev_close = mlx5_dev_close,
	//	.promiscuous_enable = mlx5_promiscuous_enable,
	//	.promiscuous_disable = mlx5_promiscuous_disable,
	//	.allmulticast_enable = mlx5_allmulticast_enable,
	//	.allmulticast_disable = mlx5_allmulticast_disable,
	//	.stats_get = mlx5_stats_get,
	//	.stats_get = mlx5_stats_get,
	//	.stats_reset = mlx5_stats_reset,
	//	.xstats_get = mlx5_xstats_get,
	//	.xstats_reset = mlx5_xstats_reset,
	//	.xstats_get_names = mlx5_xstats_get_names,
	//	.dev_infos_get = mlx5_dev_infos_get,
	//	.dev_supported_ptypes_get = mlx5_dev_supported_ptypes_get,
	//	.vlan_filter_set = mlx5_vlan_filter_set,
	//	.rx_queue_setup = mlx5_rx_queue_setup,
	//	.rx_queue_release = mlx5_rx_queue_release,
	//	.tx_queue_release = mlx5_tx_queue_release,
	//	.flow_ctrl_get = mlx5_dev_get_flow_ctrl,
	//	.flow_ctrl_set = mlx5_dev_set_flow_ctrl,
	//	.mac_addr_remove = mlx5_mac_addr_remove,
	//	.mac_addr_add = mlx5_mac_addr_add,
	//	.mac_addr_set = mlx5_mac_addr_set,
	//	.mtu_set = mlx5_dev_set_mtu,
	//	.vlan_strip_queue_set = mlx5_vlan_strip_queue_set,
	//	.vlan_offload_set = mlx5_vlan_offload_set,
	//	.reta_update = mlx5_dev_rss_reta_update,
	//	.reta_query = mlx5_dev_rss_reta_query,
	//	.rss_hash_update = mlx5_rss_hash_update,
	//	.rss_hash_conf_get = mlx5_rss_hash_conf_get,
	//	.filter_ctrl = mlx5_dev_filter_ctrl,
	//	.rx_descriptor_status = mlx5_rx_descriptor_status,
	//	.tx_descriptor_status = mlx5_tx_descriptor_status,
	//	.rx_queue_intr_enable = mlx5_rx_intr_enable,
	//	.rx_queue_intr_disable = mlx5_rx_intr_disable,
	//	.is_removed = mlx5_is_removed,
	//	.link_update = mlx5_link_update,
	};
	static struct {
		struct rte_pci_addr pci_addr; /* associated PCI address */
		uint32_t ports; /* physical ports bitfield. */
	} mlx5_dev[32];

	/**
	 * Get device index in mlx5_dev[] from PCI bus address.
	 *
	 * @param[in] pci_addr
	 *   PCI bus address to look for.
	 *
	 * @return
	 *   mlx5_dev[] index on success, -1 on failure.
	 */
	static int
	mlx5_dev_idx(struct rte_pci_addr *pci_addr)
	{
		unsigned int i;
		int ret = -1;

	       assert(pci_addr != NULL);
		for (i = 0; (i != RTE_DIM(mlx5_dev)); ++i) {
		       if ((mlx5_dev[i].pci_addr.domain == pci_addr->domain) &&
			   (mlx5_dev[i].pci_addr.bus == pci_addr->bus) &&
			   (mlx5_dev[i].pci_addr.devid == pci_addr->devid) &&
			   (mlx5_dev[i].pci_addr.function == pci_addr->function))
				return i;
			if ((mlx5_dev[i].ports == 0) && (ret == -1))
				ret = i;
		}
		return ret;
	}

	/**
	 * Verify and store value for device argument.
	 *
	 * @param[in] key
	 *   Key argument to verify.
	 * @param[in] val
	 *   Value associated with key.
	 * @param opaque
	 *   User data.
	 *
	 * @return
	 *   0 on success, negative errno value on failure.
	 */
	static int
	mlx5_args_check(const char *key, const char *val, void *opaque)
	{
		struct mlx5_mdev_dev_config *config = opaque;
		unsigned long tmp;

		errno = 0;
		tmp = strtoul(val, NULL, 0);
		if (errno) {
			WARN("%s: \"%s\" is not a valid integer", key, val);
			return errno;
		}
		if (strcmp(MLX5_RXQ_CQE_COMP_EN, key) == 0) {
			config->cqe_comp = !!tmp;
		} else if (strcmp(MLX5_TXQ_INLINE, key) == 0) {
			config->txq_inline = tmp;
		} else if (strcmp(MLX5_TXQS_MIN_INLINE, key) == 0) {
			config->txqs_inline = tmp;
		} else if (strcmp(MLX5_TXQ_MPW_EN, key) == 0) {
			config->mps = !!tmp ? config->mps : 0;
		} else if (strcmp(MLX5_TXQ_MPW_HDR_DSEG_EN, key) == 0) {
			config->mpw_hdr_dseg = !!tmp;
		} else if (strcmp(MLX5_TXQ_MAX_INLINE_LEN, key) == 0) {
			config->inline_max_packet_sz = tmp;
		} else if (strcmp(MLX5_TX_VEC_EN, key) == 0) {
			config->tx_vec_en = !!tmp;
		} else if (strcmp(MLX5_RX_VEC_EN, key) == 0) {
			config->rx_vec_en = !!tmp;
		} else {
			WARN("%s: unknown parameter", key);
			return -EINVAL;
		}
		return 0;
	}

	/**
	 * Parse device parameters.
	 *
	 * @param config
	 *   Pointer to device configuration structure.
	 * @param devargs
	 *   Device arguments structure.
	 *
	 * @return
	 *   0 on success, errno value on failure.
	 */
	static int __rte_unused
	mlx5_mdev_args(struct mlx5_mdev_dev_config *config, struct rte_devargs *devargs)
	{
		const char **params = (const char *[]){
			MLX5_RXQ_CQE_COMP_EN,
			MLX5_TXQ_INLINE,
			MLX5_TXQS_MIN_INLINE,
			MLX5_TXQ_MPW_EN,
			MLX5_TXQ_MPW_HDR_DSEG_EN,
			MLX5_TXQ_MAX_INLINE_LEN,
			MLX5_TX_VEC_EN,
			MLX5_RX_VEC_EN,
			NULL,
		};
		struct rte_kvargs *kvlist;
		int ret = 0;
		int i;

		if (devargs == NULL)
			return 0;
		/* Following UGLY cast is done to pass checkpatch. */
		kvlist = rte_kvargs_parse(devargs->args, params);
		if (kvlist == NULL)
			return 0;
		/* Process parameters. */
		for (i = 0; (params[i] != NULL); ++i) {
			if (rte_kvargs_count(kvlist, params[i])) {
				ret = rte_kvargs_process(kvlist, params[i],
							 mlx5_args_check, config);
				if (ret != 0) {
					rte_kvargs_free(kvlist);
					return ret;
				}
			}
		}
		rte_kvargs_free(kvlist);
		return 0;
	}

	static struct rte_pci_driver mlx5_mdev_pci_driver;

	/*
	 * Reserved UAR address space for TXQ UAR(hw doorbell) mapping, process
	 * local resource used by both primary and secondary to avoid duplicate
	 * reservation.
	 * The space has to be available on both primary and secondary process,
	 * TXQ UAR maps to this area using fixed mmap w/o double check.
	 */
	static void *uar_base;

	/**
	 * Reserve UAR address space for primary process.
	 *
	 * @param[in] priv
	 *   Pointer to private structure.
	 *
	 * @return
	 *   0 on success, errno value on failure.
	 */
	static int __rte_unused
	priv_uar_init_primary(struct mlx5_mdev_priv *priv)
	{
		void *addr = (void *)0;
		int i;
		const struct rte_mem_config *mcfg;
		int ret;

		if (uar_base) { /* UAR address space mapped. */
			priv->uar_base = uar_base;
			return 0;
		}
		/* find out lower bound of hugepage segments */
		mcfg = rte_eal_get_configuration()->mem_config;
		for (i = 0; i < RTE_MAX_MEMSEG && mcfg->memseg[i].addr; i++) {
			if (addr)
				addr = RTE_MIN(addr, mcfg->memseg[i].addr);
			else
				addr = mcfg->memseg[i].addr;
		}
		/* keep distance to hugepages to minimize potential conflicts. */
		addr = RTE_PTR_SUB(addr, MLX5_UAR_OFFSET + MLX5_UAR_SIZE);
		/* anonymous mmap, no real memory consumption. */
		addr = mmap(addr, MLX5_UAR_SIZE,
			    PROT_NONE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
		if (addr == MAP_FAILED) {
			ERROR("Failed to reserve UAR address space, please adjust "
			      "MLX5_UAR_SIZE or try --base-virtaddr");
			ret = ENOMEM;
			return ret;
		}
		/* Accept either same addr or a new addr returned from mmap if target
		 * range occupied.
		 */
		INFO("Reserved UAR address space: %p", addr);
		priv->uar_base = addr; /* for primary and secondary UAR re-mmap. */
		uar_base = addr; /* process local, don't reserve again. */
		return 0;
	}
	static int
	mlx5_mdev_pci_probe(struct rte_pci_driver *pci_drv,
			    struct rte_pci_device *pci_dev)
	{
		struct devx_device **list;
		struct devx_device *devx_dev;
		struct mlx5_mdev_cap *caps = NULL;
		void *attr_ctx = NULL;
		int err = 0;
		unsigned int sriov;
		unsigned int mps __rte_unused;
		unsigned int cqe_comp __rte_unused;
		unsigned int __rte_unused tunnel_en = 0;
		int idx;
		int i;
		int num_ports = 0;

		printf("oooOri device name %s\n",pci_dev->device.name);
		(void)pci_drv;
		assert(pci_drv == &mlx5_mdev_pci_driver);

		/* Get mlx5_dev[] index. */
		idx = mlx5_dev_idx(&pci_dev->addr);
		if (idx == -1) {
			ERROR("this driver cannot support any more adapters");
			return -ENOMEM;
		}
		DEBUG("using driver device index %d", idx);
		ERROR("==== using driver device index %d", idx);

		/* Save PCI address. */
		mlx5_dev[idx].pci_addr = pci_dev->addr;
		list = devx_get_device_list(&i);
		if (list == NULL) {
			assert(errno);
			if (errno == ENOSYS)
				ERROR("cannot list devices, is devx loaded ?");
			return -errno;
		}
		assert(i >= 0);
		/*
		 * For each listed device, check related sysfs entry against
		 * the provided PCI ID.
		 */
		while (i != 0) {
			struct rte_pci_addr pci_addr;

			--i;
			DEBUG("checking device \"%s\"", list[i]->name);
			ERROR("====checking device \"%s\"", list[i]->name);
			if (mlx5_mdev_device_to_pci_addr(list[i], &pci_addr))
				continue;
			if ((pci_dev->addr.domain != pci_addr.domain) ||
			    (pci_dev->addr.bus != pci_addr.bus) ||
			    (pci_dev->addr.devid != pci_addr.devid) ||
			    (pci_dev->addr.function != pci_addr.function))
				continue;
			sriov = ((pci_dev->id.device_id ==
			       PCI_DEVICE_ID_MELLANOX_CONNECTX4VF) ||
			      (pci_dev->id.device_id ==
			       PCI_DEVICE_ID_MELLANOX_CONNECTX4LXVF) ||
			      (pci_dev->id.device_id ==
			       PCI_DEVICE_ID_MELLANOX_CONNECTX5VF) ||
			      (pci_dev->id.device_id ==
			       PCI_DEVICE_ID_MELLANOX_CONNECTX5EXVF));
			switch (pci_dev->id.device_id) {
			case PCI_DEVICE_ID_MELLANOX_CONNECTX4:
				tunnel_en = 1;
				break;
			case PCI_DEVICE_ID_MELLANOX_CONNECTX4LX:
			case PCI_DEVICE_ID_MELLANOX_CONNECTX5:
			case PCI_DEVICE_ID_MELLANOX_CONNECTX5VF:
			case PCI_DEVICE_ID_MELLANOX_CONNECTX5EX:
			case PCI_DEVICE_ID_MELLANOX_CONNECTX5EXVF:
				tunnel_en = 1;
				break;
			default:
				break;
			}
			//INFO("PCI information matches, using device \"%s\""
			ERROR("===== PCI information matches, using device \"%s\""
			     " (SR-IOV: %s)",
			     list[i]->name,
			     sriov ? "true" : "false");
			attr_ctx = devx_open_device(list[i]);
			err = errno;
			break;
		}
		if (attr_ctx == NULL) {
			devx_free_device_list(list);
			switch (err) {
			case 0:
				ERROR("cannot access device, is devx loaded?");
				return -ENODEV;
			case EINVAL:
				ERROR("cannot use device, are drivers up to date?");
				return -EINVAL;
			}
			assert(err >0);
			return -err;
		}
		devx_dev = list[i];

		DEBUG("device opened");
		ERROR("==== device opened");

		/* Query and verify device capabilities */
		caps = rte_zmalloc("HCA caps", sizeof(*caps), RTE_CACHE_LINE_SIZE);
		if (caps == NULL) {
			ERROR("failed to allocate memory for HCA capabilities");
			err = ENOMEM;
			goto error;
		}
		if (mlx5_mdev_query_hca_cap(attr_ctx, caps) ||
		    mlx5_mdev_check_hca_cap(caps)) {
			err = EFAULT;
			goto error;
		}
		/*
		 * Multi-packet send is supported by ConnectX-4 Lx PF as well
		 * as all ConnectX-5 devices.
		 */
		if (MLX5_CAP_ETH(caps, multi_pkt_send_wqe)) {
			if (MLX5_CAP_ETH(caps, enhanced_multi_pkt_send_wqe)) {
				DEBUG("Enhanced MPW is supported");
				ERROR("======== Enhanced MPW is supported");
				mps = MLX5_MPW_ENHANCED;
			} else {
				DEBUG("MPW is supported");
				ERROR("======= MPW is supported");
				mps = MLX5_MPW;
			}
		} else {
			DEBUG("MPW isn't supported");
			ERROR("======= MPW isn't supported");
			mps = MLX5_MPW_DISABLED;
		}
		if (RTE_CACHE_LINE_SIZE == 128 &&
		    !(MLX5_CAP_GEN(caps, cqe_compression) &&
		      MLX5_CAP_GEN(caps, cqe_compression_128b))) // FIXME ? is this the correct check ?
			cqe_comp = 0;
		else
			cqe_comp = 1;
		num_ports = MLX5_CAP_GEN(caps, num_ports);
		INFO("%u port(s) detected", num_ports);
		ERROR("======= %u port(s) detected", num_ports);

		for (i = 0; i < num_ports; i++) {
			char name[RTE_ETH_NAME_MAX_LEN];
			int len;
			uint32_t port = i + 1; /* ports are indexed from one */
			uint32_t test = (1 << i);
			struct mlx5_mdev_priv *priv = NULL;
			void *dev_ctx = NULL;
			struct rte_eth_dev *eth_dev;
			struct ether_addr mac;
			//uint16_t num_vfs = 0; TODO: handle me
			uint8_t port_admin_state;
			uint8_t port_state;
			struct mlx5_mdev_dev_config config = {
				.cqe_comp = cqe_comp,
				.mps = mps,
				.tunnel_en = tunnel_en,
				.tx_vec_en = 1,
				.rx_vec_en = 1,
				.mpw_hdr_dseg = 0,
				.txq_inline = MLX5_ARG_UNSET,
				.txqs_inline = MLX5_ARG_UNSET,
				.inline_max_packet_sz = MLX5_ARG_UNSET,
			};

		len = snprintf(name, sizeof(name), PCI_PRI_FMT,
			 pci_dev->addr.domain, pci_dev->addr.bus,
			 pci_dev->addr.devid, pci_dev->addr.function);
		if (num_ports > 1)
			snprintf(name + len, sizeof(name), " port %u", i);

		mlx5_dev[idx].ports |= test;
		mlx5_dev[idx].ports |= test;
#if 0
		if (rte_eal_process_type() == RTE_PROC_SECONDARY) {
			eth_dev = rte_eth_dev_attach_secondary(name);
			if (eth_dev == NULL) {
				ERROR("can not attach rte ethdev");
				err = ENOMEM;
				goto error;
			}
			eth_dev->device = &pci_dev->device;
			eth_dev->dev_ops = &mlx5_dev_sec_ops;
			priv = eth_dev->data->dev_private;
			err = priv_uar_init_secondary(priv);
			if (err < 0) {
				err = -err;
				goto error;
			}
			/* Receive command fd from primary process */
			err = priv_socket_connect(priv);
			if (err < 0) {
				err = -err;
				goto error;
			}
			/* Remap UAR for Tx queues. */
			err = priv_tx_uar_remap(priv, err);
			if (err)
				goto error;
			/*
			 * Ethdev pointer is still required as input since
			 * the primary device is not accessible from the
			 * secondary process.
			 */
			eth_dev->rx_pkt_burst =
				priv_select_rx_function(priv, eth_dev);
			eth_dev->tx_pkt_burst =
				priv_select_tx_function(priv, eth_dev);
			continue;
		}
#endif
		DEBUG("using port %u (%08" PRIx32 ")", port, test);
		ERROR("======== using port %u (%08" PRIx32 ")", port, test);

		dev_ctx = devx_open_device(devx_dev);
		if (dev_ctx == NULL) {
			err = ENODEV;
			goto port_error;
		}

		if (MLX5_CAP_GEN(caps, port_type) != MLX5_CAP_PORT_TYPE_ETH) {
			ERROR("port %d is not configured in Ethernet mode",
			      port);
			err = EINVAL;
			goto port_error;
		}

		if (mlx5_mdev_query_vport_state(dev_ctx,
		                                &port_admin_state,
		                                &port_state) ||
		    port_admin_state  != MLX5_QUERY_VPORT_STATE_OUT_STATE_UP)
			// DEBUG("port %d is not active: (%d)",
			ERROR("==== port %d is not active: (%d)",
			      port, port_admin_state);

		/* from rte_ethdev.c */
		priv = rte_zmalloc("ethdev private structure",
				   sizeof(*priv),
				   RTE_CACHE_LINE_SIZE);
		if (priv == NULL) {
			ERROR("priv allocation failure");
			err = ENOMEM;
			goto port_error;
		}

		priv->db_page = NULL;
		priv->ctx = dev_ctx;

		strncpy(priv->ibdev_path, devx_dev->ibdev_path,
			sizeof(priv->ibdev_path));

		/* Allocate protection and transport domains. */
		err = mlx5_mdev_alloc_pd(priv);
		if (err) {
			ERROR("PD allocation failure");
			goto port_error;
		}
		err = mlx5_mdev_alloc_td(priv);
		if (err) {
			ERROR("TD allocation failure");
			goto port_error;
		}
		mlx5_dev[idx].ports |= test;

		//priv->device_attr = device_attr; TODO : add me
		priv->port = port;
		priv->mtu = ETHER_MTU;
		rte_memcpy(&priv->caps, caps, sizeof(priv->caps));
		err = mlx5_mdev_args(&config, pci_dev->device.devargs);
		if (err) {
			ERROR("failed to process device arguments: %s",
			      strerror(err));
			goto port_error;
		}

		config.hw_csum = !!(MLX5_CAP_ETH(&(priv->caps), csum_cap));
		// DEBUG("checksum offloading is %ssupported",
		ERROR("==== checksum offloading is %ssupported",
		      (config.hw_csum ? "" : "not "));
		config.hw_csum_l2tun = !!(MLX5_CAP_ETH(&(priv->caps), tunnel_stateless_vxlan));
		//DEBUG("Rx L2 tunnel checksum offloads are %ssupported",
		ERROR("======= Rx L2 tunnel checksum offloads are %ssupported",
				      (config.hw_csum_l2tun ? "" : "not "));

#ifdef HAVE_IBV_DEVICE_COUNTERS_SET_SUPPORT // TODO: Revive me
		config.flow_counter_en =
			!!(MLX5_CAP_HCA(priv,max_flow_counter_31_16) |
			   MLX5_CAP_HCA(priv,max_flow_counter_15_0));

		// !!(device_attr.max_counter_sets);
		//mlx5_glue->describe_counter_set(ctx, 0, &cs_desc); TODO: Revive me
		DEBUG("counter type = %d, num of cs = %ld, attributes = %d",
		      cs_desc.counter_type, cs_desc.num_of_cs,
		      cs_desc.attributes);
#endif
		/* was device_attr_ex.rss_caps.max_rwq_indirection_table_size
		 * derived from 1 << MLX5_CAP_GEN(&priv->caps, log_max_rqt_size), see
		 * <kernel>/drivers/infiniband/hw/mlx5/main.c
		 */
		config.ind_table_max_size = 1 << (MLX5_CAP_GEN(&priv->caps, log_max_rqt_size));

		/* Remove this check once DPDK supports larger/variable
		 * indirection tables. */
		if (config.ind_table_max_size >
				(unsigned int)ETH_RSS_RETA_SIZE_512)
			config.ind_table_max_size = ETH_RSS_RETA_SIZE_512;
		//DEBUG("maximum RX indirection table size is %u",
		ERROR("========= maximum RX indirection table size is %u",
		      config.ind_table_max_size);

		config.hw_vlan_strip = !!(MLX5_CAP_ETH(&priv->caps, vlan_cap));
		//DEBUG("VLAN stripping is %ssupported",
		ERROR("========= VLAN stripping is %ssupported",
		      (config.hw_vlan_strip ? "" : "not "));

		config.hw_fcs_strip = !!(MLX5_CAP_ETH(&priv->caps, scatter_fcs));
		//DEBUG("FCS stripping configuration is %ssupported",
		ERROR("========= FCS stripping configuration is %ssupported",
		      (config.hw_fcs_strip ? "" : "not "));

		config.hw_padding = !!(MLX5_CAP_GEN(&priv->caps, end_pad));
		//DEBUG("hardware RX end alignment padding is %ssupported",
		ERROR("========= hardware RX end alignment padding is %ssupported",
				      (config.hw_padding ? "" : "not "));

		config.sriov = !!(MLX5_CAP_GEN(&priv->caps, eth_virt));

		config.tso = !!(MLX5_CAP_ETH(&priv->caps, max_lso_cap));
		if (config.tso)
			config.tso_max_payload_sz =
					1 << (MLX5_CAP_ETH(&priv->caps, max_lso_cap));
		if (config.mps && !mps) {
			ERROR("multi-packet send not supported on this device"
			      " (" MLX5_TXQ_MPW_EN ")");
			err = ENOTSUP;
			goto port_error;
		}
		//INFO("%sMPS is %s",
		ERROR("========= %sMPS is %s",
		     config.mps == MLX5_MPW_ENHANCED ? "Enhanced " : "",
		     config.mps != MLX5_MPW_DISABLED ? "enabled" : "disabled");

		if (config.cqe_comp && !cqe_comp) {
			WARN("Rx CQE compression isn't supported");
			ERROR("========= Rx CQE compression isn't supported");
			config.cqe_comp = 0;
		}

		err = priv_uar_init_primary(priv); // FIXME
		if (err) {
			ERROR(" =========%s: %d", __FUNCTION__, __LINE__);
			goto port_error;
		}
		err = devx_alloc_uar(priv->ctx, &priv->uar.index, &priv->uar.uar);
		if (err) {
			ERROR(" =========%s: %d", __FUNCTION__, __LINE__);
			goto port_error;
		}
		/* Configure the first MAC address by default. */
		if (mdev_priv_get_mac(priv, &mac.addr_bytes)) {
			ERROR("cannot get MAC address, is mlx5_en loaded?"
			      " (errno: %s)", strerror(errno));
			err = ENODEV;
			goto port_error;
		}

		//INFO("port %u MAC address is %02x:%02x:%02x:%02x:%02x:%02x",
		ERROR("========= port %u MAC address is %02x:%02x:%02x:%02x:%02x:%02x",
		     priv->port,
		     mac.addr_bytes[0], mac.addr_bytes[1],
		     mac.addr_bytes[2], mac.addr_bytes[3],
		     mac.addr_bytes[4], mac.addr_bytes[5]);
#ifndef NDEBUG
		{
			char ifname[IF_NAMESIZE];
			if (priv_get_ifname(priv, &ifname) == 0)
				//DEBUG("port %u ifname is \"%s\"",
				ERROR("========= port %u ifname is \"%s\"",
				      priv->port, ifname);
			else

				//DEBUG("port %u ifname is unknown", priv->port);
				ERROR("========= port %u ifname is unknown", priv->port);
		}
#endif
		/* Get actual MTU if possible. */
		priv_get_mtu(priv, &priv->mtu);
		ERROR("======== port %u MTU is %u\n", priv->port, priv->mtu);
		DEBUG("port %u MTU is %u", priv->port, priv->mtu);

		eth_dev = rte_eth_dev_allocate(name);
		if (eth_dev == NULL) {
			ERROR("can not allocate rte ethdev");
			err = ENOMEM;
			goto port_error;
		}
		eth_dev->data->dev_private = priv;
		eth_dev->data->mac_addrs = priv->mac;
		eth_dev->device = &pci_dev->device;
		rte_eth_copy_pci_info(eth_dev, pci_dev);
		eth_dev->device->driver = &mlx5_mdev_pci_driver.driver;
		/*
		 * Initialize burst functions to prevent crashes before link-up.
		 */
		eth_dev->tx_pkt_burst =
				priv_select_tx_function(priv, eth_dev);
//		eth_dev->rx_pkt_burst = removed_rx_burst;
//		eth_dev->tx_pkt_burst = removed_tx_burst;
		priv->dev = eth_dev;
		eth_dev->dev_ops = &mlx5_mdev_dev_ops; // TODO: Return: &mlx5_mdev_ops;
		/* Register MAC address. */
		claim_zero(mlx5_mdev_mac_addr_add(eth_dev, &mac, 0, 0));
//		TAILQ_INIT(&priv->flows);
//		TAILQ_INIT(&priv->ctrl_flows);

		/* Bring Ethernet device up. */
		DEBUG("forcing Ethernet interface up");
		priv_set_flags(priv, ~IFF_UP, IFF_UP);
		/* Store device configuration on private structure. */
		priv->config = config;
		continue;

//FIXME: should destroy objects
port_error:
		if (priv) { // TODO: Add dealloc routines
#if 0
			if (priv->td.td_object)
				mlx5_mdev_dealloc_td(priv->td.td_object);
			if (priv->pd.pd_object)
				mlx5_mdev_dealloc_pd(priv->pd.pd_object);
#endif
			rte_free(priv);
		}
		if (dev_ctx)
			claim_zero(devx_close_device(dev_ctx));
		break;
	}

	/*
	 * XXX if something went wrong in the loop above, there is a resource
	 * leak (ctx, pd, priv, dpdk ethdev) but we can do nothing about it as
	 * long as the dpdk does not provide a way to deallocate a ethdev and a
	 * way to enumerate the registered ethdevs to free the previous ones.
	 */

	/* no port found, complain */
	if (!mlx5_dev[idx].ports) {
		ERROR(" =========%s: %d", __FUNCTION__, __LINE__);
		err = ENODEV;
		goto error;
	}

error:
	rte_free(caps);
	if (attr_ctx)
		claim_zero(devx_close_device(attr_ctx));
	if (list)
		devx_free_device_list(list);
	ERROR(" =========%s: %d ret %d", __FUNCTION__, __LINE__, err);
	assert(err >= 0);
	return -err;
}
#if 0
static int mlx5_mdev_init(struct rte_eth_dev *edev)
{

	struct mlx5_mdev_priv *priv = edev->data->dev_private;
	struct rte_pci_device *pdev = RTE_ETH_DEV_TO_PCI(edev);

	priv->edev = edev;

	edev->data->dev_private = priv;

	edev->dev_ops = &mlx5_mdev_dev_ops;

	priv->base_addr = (void *)pdev->mem_resource[0].addr;
	priv->cache_line_size = RTE_CACHE_LINE_SIZE;
	priv->page_size = 4096; //oooOri todo change to page size
	priv->dev_context = mdev_open_device(priv->edev,
						priv->base_addr,
						alloc_pinned);
	struct mdev_cq_attr cq_attr = {0};
	struct mdev_eq_attr eq_attr = {0};
	struct mdev_tis_attr tis_attr = {0};

	cq_attr.cqe = 64;
	cq_attr.ctx = priv->dev_context;
	eq_attr.ctx = priv->dev_context;
	eq_attr.eqe = 64;
	struct mdev_eq * eq =
		mlx5_mdev_create_eq(priv, &eq_attr);
	cq_attr.eqn = eq->eqn;
	struct mdev_cq * cq =
			mlx5_mdev_create_cq(priv, &cq_attr);
	tis_attr.ctx = priv->dev_context;
	tis_attr.td = priv->dev_context->td;
	struct mdev_tis *tis = mlx5_mdev_create_tis(priv, &tis_attr);


	struct mdev_sq_attr sq_attr = {0};
	sq_attr.cqn = cq->cqn;
	sq_attr.ctx = priv->dev_context;
	sq_attr.nelements = 64;
	sq_attr.tisn = tis->tisn;
	sq_attr.wq.pd = priv->dev_context->pd;
	struct mdev_sq *sq = mlx5_mdev_create_sq(priv,&sq_attr);

	return 0;
}
#endif
#if 0
static int mlx5_mdev_uninit(struct rte_eth_dev *edev)
{

	edev = edev;


	return 0;
}
#endif
#if 0
static int mlx5_mdev_pci_probe(__rte_unused struct rte_pci_driver *pci_drv,
			   struct rte_pci_device *pci_dev)
{

	struct rte_eth_dev *eth_dev;
	int ret=0;
	
	assert(pci_drv == &mlx5_mdev_driver);

	eth_dev = rte_eth_dev_pci_allocate(pci_dev,
		sizeof(struct mlx5_mdev_priv));

	if (!eth_dev) {
		ERROR("rte_eth_dev_pci_allocate Failed");
		return -ENOMEM;
	}
	ret = mlx5_mdev_init(eth_dev);
	if (ret) {
		ERROR("mlx5_mdev_init Failed (%d)", ret);
		rte_eth_dev_pci_release(eth_dev);
	}
	return ret;
}
#endif
#if 0
	struct devx_device **device_list;
	int num_devices;
	void *device = NULL;
	int err = 0;
	int i;
	int32_t tunnel_en =0;
	int32_t sriov;
	//uint8_t mac_address[6];

	device_list = devx_get_device_list(&num_devices);	
	//return rte_eth_dev_pci_generic_probe(pdev, sizeof(struct mlx5_mdev_priv),
	//		mlx5_mdev_init);
	while (num_devices !=0) {
		struct rte_pci_addr pci_addr;

		num_devices--;
		if (devx_device_to_pci_addr(device_list[num_devices],
						&pci_addr))
			continue;
		if ((pci_dev->addr.domain != pci_addr.domain) ||
		    (pci_dev->addr.bus != pci_addr.bus) ||
		    (pci_dev->addr.devid != pci_addr.devid) ||
		    (pci_dev->addr.function != pci_addr.function))
			continue;
		sriov = ((pci_dev->id.device_id ==
		       PCI_DEVICE_ID_MELLANOX_CONNECTX4VF) ||
		      (pci_dev->id.device_id ==
		       PCI_DEVICE_ID_MELLANOX_CONNECTX4LXVF) ||
		      (pci_dev->id.device_id ==
		       PCI_DEVICE_ID_MELLANOX_CONNECTX5VF) ||
		      (pci_dev->id.device_id ==
		       PCI_DEVICE_ID_MELLANOX_CONNECTX5EXVF));
		switch (pci_dev->id.device_id) {
		case PCI_DEVICE_ID_MELLANOX_CONNECTX4:
			tunnel_en = 1;
			break;
		case PCI_DEVICE_ID_MELLANOX_CONNECTX4LX:
		case PCI_DEVICE_ID_MELLANOX_CONNECTX5:
		case PCI_DEVICE_ID_MELLANOX_CONNECTX5VF:
		case PCI_DEVICE_ID_MELLANOX_CONNECTX5EX:
		case PCI_DEVICE_ID_MELLANOX_CONNECTX5EXVF:
			tunnel_en = 1;
			break;
		default:
			break;
		}
		INFO("PCI information matches, using device \\%s\\(SR-IOV: %s)",
		     device_list[num_devices]->name,
		     sriov ? "true" : "false");
		device = devx_open_device(device_list[num_devices]);
		err = errno;
		
		if (device == NULL) {
			devx_free_device_list(device_list);
			switch (err) {
			case 0:
				ERROR("cannot access device");
				return -ENODEV;
			case EINVAL:
				ERROR("cannot use device");
				return -EINVAL;
		
			assert(err >0);
			return -err;
			}
		}
		for (i = 0; i < 1; i++) {
			char name[RTE_ETH_NAME_MAX_LEN];
			//int len;
			//uint32_t port = i+1; /* ports are indexed from one */
			struct mlx5_mdev_priv *priv = NULL;
			//struct ether_addr mac;
			struct rte_eth_dev *eth_dev;
		
			snprintf(name, sizeof(name), PCI_PRI_FMT,
				pci_dev->addr.domain, pci_dev->addr.bus,
				pci_dev->addr.devid, pci_dev->addr.function);

			priv = rte_zmalloc("ethdev private structure",
						sizeof(*priv),
						RTE_CACHE_LINE_SIZE);

			if (priv == NULL) {
				ERROR("priv alloaction failure");
				err = ENOMEM;
				goto port_error;
			}	
			eth_dev = rte_eth_dev_allocate(name);
			priv->edev = eth_dev;
			priv->dev = device;
			mlx5_mdev_alloc_pd(priv);
			err = devx_alloc_uar(priv->dev, &priv->uar_index,
					(void *)&priv->uar); 
			
			if (eth_dev == NULL) {
				ERROR("can not allocate rte ethdev");
				err = ENOMEM;
				goto port_error;
			}
			//priv_get_mac(priv, &mac_address);
			//get_mac_address(mac_address);
			//eth_dev->data->dev_private = priv;
			//eth_dev->data->mac_addrs = priv->mac;
			//eth_dev->device = &pci_dev->device;
			//rte_eth_copy_pci_info(eth_dev, pci_dev);
			//eth_dev->device->driver = &mlx5_driver.driver;
			continue;		
port_error:
			if(priv)
				rte_free(priv);
			if(device)
				devx_close_device(device);
			goto error;	
		}
	}
	return 0;
error:
	return -err;
	return 0;
}
#endif
#if 0
static int mlx5_mdev_pci_remove(struct rte_pci_device *pci_dev)
{
	return rte_eth_dev_pci_generic_remove(pci_dev, mlx5_mdev_uninit);
}
#endif
static const struct rte_pci_id mlx5_mdev_pci_id_map[] = {
	{
		RTE_PCI_DEVICE(PCI_VENDOR_ID_MELLANOX,
			       PCI_DEVICE_ID_MELLANOX_CONNECTX4)
	},
	{
		RTE_PCI_DEVICE(PCI_VENDOR_ID_MELLANOX,
			       PCI_DEVICE_ID_MELLANOX_CONNECTX4VF)
	},
	{
		RTE_PCI_DEVICE(PCI_VENDOR_ID_MELLANOX,
			       PCI_DEVICE_ID_MELLANOX_CONNECTX4LX)
	},
	{
		RTE_PCI_DEVICE(PCI_VENDOR_ID_MELLANOX,
			       PCI_DEVICE_ID_MELLANOX_CONNECTX4LXVF)
	},
	{
		RTE_PCI_DEVICE(PCI_VENDOR_ID_MELLANOX,
			       PCI_DEVICE_ID_MELLANOX_CONNECTX5)
	},
	{
		RTE_PCI_DEVICE(PCI_VENDOR_ID_MELLANOX,
			       PCI_DEVICE_ID_MELLANOX_CONNECTX5VF)
	},
	{
		RTE_PCI_DEVICE(PCI_VENDOR_ID_MELLANOX,
			       PCI_DEVICE_ID_MELLANOX_CONNECTX5EX)
	},
	{
		RTE_PCI_DEVICE(PCI_VENDOR_ID_MELLANOX,
			       PCI_DEVICE_ID_MELLANOX_CONNECTX5EXVF)
	},
	{
		.vendor_id = 0
	}
};

static struct rte_pci_driver mlx5_mdev_pci_driver = {
	.driver = {
		.name = "MLX5_MDEV_DRIVER" 
	},
	.id_table	= mlx5_mdev_pci_id_map,
	.drv_flags	= RTE_PCI_DRV_INTR_LSC | RTE_PCI_DRV_INTR_RMV,
	.probe		= mlx5_mdev_pci_probe,
};

/**
 * Driver initialization routine.
 */
RTE_INIT(rte_mlx5_mdev_pmd_init);
static void
rte_mlx5_mdev_pmd_init(void)
{
	/* Build the static table for ptype conversion. */
//	mlx5_mdev_set_ptype_table();

	/* Match the size of Rx completion entry to the size of a cacheline. */
	if (RTE_CACHE_LINE_SIZE == 128)
		setenv("MLX5_MDEV_CQE_SIZE", "128", 0);

	rte_pci_register(&mlx5_mdev_pci_driver);
}

RTE_PMD_EXPORT_NAME(net_mlx5_mdev, __COUNTER__);
RTE_PMD_REGISTER_PCI_TABLE(net_mlx5_mdev, mlx5_mdev_pci_id_map);
RTE_PMD_REGISTER_KMOD_DEP(net_mlx5_mdev, "* ib_uverbs & mlx5_core");
