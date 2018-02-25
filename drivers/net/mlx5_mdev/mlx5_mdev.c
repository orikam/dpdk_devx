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

#include <rte_io.h>
#include <rte_pci.h>
#include <rte_log.h>
#include <rte_malloc.h>
#include <rte_memzone.h>

#include <rte_ether.h>
#include <rte_ethdev.h>
#include <rte_ethdev_pci.h>
#include <rte_eth_ctrl.h>

#include "devx.h"
#include "mlx5_mdev_utils.h"
#include "mdev_lib.h"
#include "mlx5_mdev_priv.h"
#include "mlx5_mdev.h"


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

static int mlx5_mdev_dev_start(struct rte_eth_dev *edev)
{
	edev = edev;

	return 0;
}

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
	.dev_start		= mlx5_mdev_dev_start,
	.dev_infos_get		= mlx5_mdev_infos_get,
};
#if 0
static int mlx5_mdev_init(struct rte_eth_dev *edev)
{

	struct mlx5_mdev_priv *priv = edev->data->dev_private;
	struct rte_pci_device *pdev = RTE_ETH_DEV_TO_PCI(edev);
	printf("oooOri in mlx5_mdev_init start\n");

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
	printf("ooooOri in mlx5_mdev_init after cq = %x, %x, %x, %x\n", eq->eqn, cq->cqn, tis->tisn, sq->wq.dbr_addr);

	printf("ooooOri in mlx5_mdev_init end\n");
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
static int mlx5_mdev_pci_probe(__rte_unused struct rte_pci_driver *pci_drv,
			   struct rte_pci_device *pci_dev)
{
	printf("ooooOri pci probe\n");
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
	printf("oooOri num of devices %d, %p\n",num_devices, device_list);
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
			printf("oooOri uar res = %x, index = %x, tunnel = %d\n",
				err,
				priv->uar_index, tunnel_en);
			
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
	printf("oooOri end of pci probe\n");
	return 0;
error:
	return -err;
}
#if 0
static int mlx5_mdev_pci_remove(struct rte_pci_device *pci_dev)
{
	return rte_eth_dev_pci_generic_remove(pci_dev, mlx5_mdev_uninit);
}
#endif
static const struct rte_pci_id mlx5_mdev_pci_id_map[] = {
	{ RTE_PCI_DEVICE(0x15b3, 0x1014) }, /* ConnectX-4   VF */
	{ RTE_PCI_DEVICE(0x15b3, 0x1016) }, /* ConnectX-4Lx VF */
	{ RTE_PCI_DEVICE(0x15b3, 0x1018) }, /* ConnectX-5   VF */
	{ RTE_PCI_DEVICE(0x15b3, 0x101a) }, /* ConnectX-5Ex VF */
	{ RTE_PCI_DEVICE(0x15b3, 0x1017) }, /* ConnectX-5 */
	{ .vendor_id = 0, /* sentinel */ },
};

static struct rte_pci_driver mlx5_mdev_pci_driver = {
	.id_table	= mlx5_mdev_pci_id_map,
	.drv_flags	= RTE_PCI_DRV_INTR_LSC | RTE_PCI_DRV_INTR_RMV,//RTE_PCI_DRV_NEED_MAPPING,
	.probe		= mlx5_mdev_pci_probe,
	//.remove		= mlx5_mdev_pci_remove,
};

RTE_PMD_REGISTER_PCI(net_mlx5_mdev, mlx5_mdev_pci_driver);
RTE_PMD_REGISTER_PCI_TABLE(net_mlx5_mdev, mlx5_mdev_pci_id_map);
//RTE_PMD_REGISTER_KMOD_DEP(net_mlx5_mdev, "* ib_uverbs | mlx5_core | * igb_uio | uio_pci_generic | vfio-pci");
RTE_PMD_REGISTER_KMOD_DEP(net_mlx5_mdev, "* ib_uverbs & mlx5_core");
