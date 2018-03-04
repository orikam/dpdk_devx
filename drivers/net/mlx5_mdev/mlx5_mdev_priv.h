/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2015 Mellanox.
 */

#ifndef MLX5_MDEV_PRIV_H_
#define MLX5_MDEV_PRIV_H_
#include <sys/ioctl.h>
#include <net/if.h>
#include "mlx5_mdev_defs.h"
#include "mdev_lib.h"
#include "devx.h"

enum {
	PCI_VENDOR_ID_MELLANOX = 0x15b3,
};

enum {
	PCI_DEVICE_ID_MELLANOX_CONNECTX4 = 0x1013,
	PCI_DEVICE_ID_MELLANOX_CONNECTX4VF = 0x1014,
	PCI_DEVICE_ID_MELLANOX_CONNECTX4LX = 0x1015,
	PCI_DEVICE_ID_MELLANOX_CONNECTX4LXVF = 0x1016,
	PCI_DEVICE_ID_MELLANOX_CONNECTX5 = 0x1017,
	PCI_DEVICE_ID_MELLANOX_CONNECTX5VF = 0x1018,
	PCI_DEVICE_ID_MELLANOX_CONNECTX5EX = 0x1019,
	PCI_DEVICE_ID_MELLANOX_CONNECTX5EXVF = 0x101a,
};

/* MPW mode. */
enum mlx5_mpw_mode {
	MLX5_MPW_DISABLED,
	MLX5_MPW,
	MLX5_MPW_ENHANCED, /* Enhanced Multi-Packet Send WQE, a.k.a MPWv2. */
};

/* Default PMD specific parameter value. */
#define MLX5_ARG_UNSET (-1)

/*
 * Device configuration structure.
 *
 * Merged configuration from:
 *
 *  - Device capabilities,
 *  - User device parameters disabled features.
 */
struct mlx5_mdev_dev_config {
	unsigned int hw_csum:1; /* Checksum offload is supported. */
	unsigned int hw_csum_l2tun:1; /* Same for L2 tunnels. */
	unsigned int hw_vlan_strip:1; /* VLAN stripping is supported. */
	unsigned int hw_fcs_strip:1; /* FCS stripping is supported. */
	unsigned int hw_padding:1; /* End alignment padding is supported. */
	unsigned int sriov:1; /* This is a VF or PF with VF devices. */
	unsigned int mps:2; /* Multi-packet send supported mode. */
	unsigned int tunnel_en:1; /* Whether tunnel is supported. */
	unsigned int flow_counter_en:1; /* Whether flow counter is supported. */
	unsigned int cqe_comp:1; /* CQE compression is enabled. */
	unsigned int tso:1; /* Whether TSO is supported. */
	unsigned int tx_vec_en:1; /* Tx vector is enabled. */
	unsigned int rx_vec_en:1; /* Rx vector is enabled. */
	unsigned int mpw_hdr_dseg:1; /* Enable DSEGs in the title WQEBB. */
	unsigned int tso_max_payload_sz; /* Maximum TCP payload for TSO. */
	unsigned int ind_table_max_size; /* Maximum indirection table size. */
	int txq_inline; /* Maximum packet size for inlining. */
	int txqs_inline; /* Queue number threshold for inlining. */
	int inline_max_packet_sz; /* Max packet size for inlining. */
};

struct mlx5_mdev_db_page {
	const struct rte_memzone *rte_mz;
	int num_db;
	int use_cnt;
	uint32_t free_records[];
};

struct mdev_cq_attr {
	void *dev;
	uint32_t cqe; /* Minimum number of entries required for CQ */
	uint32_t create_flags;
	uint32_t eqn;
};

struct mdev_tis_attr {
	void *dev;
	uint32_t td;
};

struct mdev_eq_attr {
	void *dev;
	uint32_t eqe; /* Minimum number of entries required for CQ */
	uint32_t uar;
};

struct mdev_wq_attr {
	uint8_t wq_type;
	uint8_t page_offset;
	uint32_t pd;
	uint32_t uar_page;
	uint64_t dbr_addr;
	uint32_t hw_counter;
	uint32_t sw_counter;
	uint8_t wq_stride;
	uint8_t page_size;
	uint8_t wq_size;


};

struct mdev_sq_attr {
	void *dev;
	uint32_t nelements;
	uint8_t rlkey;
	uint8_t fre;
	uint8_t inline_mode;
	uint32_t cqn;
	uint32_t tisn;
	uint32_t uar;
	struct mdev_wq_attr wq;
};

struct mdev_pd {
	int32_t pd;
	struct devx_obj_handle *pd_object;
};
struct mdev_td {
	int32_t td;
	struct devx_obj_handle *td_object;
};
struct mdev_uar {
	uint32_t uar_index;
	uint64_t *uar;
};
struct mlx5_mdev_priv {
	struct rte_eth_dev *dev;
	char ibdev_path[256]; /* IB device path for secondary */
//	void	*base_addr;
	void *ctx;
	struct mlx5_mdev_db_page *db_page;
	struct mdev_pd pd;
	struct mdev_td td;
	int32_t page_size;
	int32_t cache_line_size;
	struct ether_addr mac[MLX5_MAX_MAC_ADDRESSES]; /* MAC addresses. */
	struct mdev_uar uar;
	void *uar_base; /* Reserved address space for UAR mapping */
	uint16_t mtu; /* Configured MTU. */
	uint8_t port; /* Physical port number. */
	rte_spinlock_t lock; /* Lock for control functions. */
	struct mlx5_mdev_dev_config config;
};

struct mdev_cq {
	void *dev;
	const struct rte_memzone *buf;
	uint64_t dbrec;
	uint32_t cqe_size;
	uint32_t uar_page;
	uint32_t cqn;
	uint32_t cons_index;
	uint32_t eqn;
	uint32_t ncqe;
	struct devx_obj_handle *object;
};

struct mdev_eq {
	void *dev;
	const struct rte_memzone *buf;
	uint64_t dbrec;
	uint32_t eqe_size;
	uint32_t uar_page;
	uint32_t cons_index;
	uint32_t eqn;
	uint32_t neqe;
	struct devx_obj_handle *object;
};

struct mdev_tis {
	void *dev;
	uint32_t td;
	uint8_t priority;
	uint32_t tisn;
	struct devx_obj_handle *object;
};

struct mdev_wq {
	uint8_t wq_type;
	uint32_t pd;
	uint32_t uar_page;
	uint32_t dbr_addr;
	uint32_t hw_counter;
	uint32_t sw_counter;
	uint8_t stride_sz; /* The size of a WQ stride equals 2^log_wq_stride. */
	uint8_t page_sz; /* The size of a WQ stride equals 2^log_wq_stride. */
	uint8_t sz; /* The size of a WQ stride equals 2^log_wq_stride. */
	const struct rte_memzone *buf;

};


struct mdev_sq {
	void *dev;
	uint32_t cqn;
	uint32_t tisn;
	struct mdev_wq wq;
	struct devx_obj_handle *object;
};

int64_t mlx5_get_dbrec(struct mlx5_mdev_priv *priv);


struct mdev_eq *
mlx5_mdev_create_eq(struct mlx5_mdev_priv *priv,
		    struct mdev_eq_attr *eq_attr);
struct mdev_cq *
mlx5_mdev_create_cq(struct mlx5_mdev_priv *priv,
		    struct mdev_cq_attr *cq_attr);

struct mdev_tis *
mlx5_mdev_create_tis(struct mlx5_mdev_priv *priv,
		    struct mdev_tis_attr *tis_attr);

struct mdev_sq *
mlx5_mdev_create_sq(struct mlx5_mdev_priv *priv,
		    struct mdev_sq_attr *sq_attr);

int mlx5_mdev_alloc_pd(struct mlx5_mdev_priv *priv);
int mlx5_mdev_alloc_td(struct mlx5_mdev_priv *priv);
int priv_get_mac(struct mlx5_mdev_priv *, uint8_t (*)[ETHER_ADDR_LEN]);
int
priv_ifreq(const struct mlx5_mdev_priv *priv, int req, struct ifreq *ifr);

/* mlx5_ethdev.c */

struct mlx5_mdev_priv *mlx5_get_priv(struct rte_eth_dev *dev);
int priv_get_ifname(const struct mlx5_mdev_priv *, char (*)[IFNAMSIZ]);
int priv_ifreq(const struct mlx5_mdev_priv *, int req, struct ifreq *);
int priv_is_ib_cntr(const char *);
int priv_get_cntr_sysfs(struct mlx5_mdev_priv *, const char *, uint64_t *);
int priv_get_num_vfs(struct mlx5_mdev_priv *, uint16_t *);
int priv_get_mtu(struct mlx5_mdev_priv *, uint16_t *);
int priv_set_flags(struct mlx5_mdev_priv *, unsigned int, unsigned int);

/*mlx5_mac.c */
int
priv_get_mac(struct mlx5_mdev_priv *priv, uint8_t (*mac)[ETHER_ADDR_LEN]);

#endif

