/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2015 Mellanox.
 */

#ifndef MLX5_MDEV_PRIV_H_
#define MLX5_MDEV_PRIV_H_
#include <sys/ioctl.h>
#include <net/if.h>
#include <sys/queue.h>
#include <rte_ether.h>

#include "mlx5_mdev_defs.h"
#include "mdev_lib.h"
#include "devx.h"
#include "mlx5_mdev_prm.h"
#include "mlx5_mdev_rxtx.h"
enum {
	PCI_VENDOR_ID_MELLANOX = 0x15b3,
};

enum {
	MLX5_RCV_DBR	= 0,
	MLX5_SND_DBR	= 1,
};

enum {
	MLX5_INLINE_SEG	= 0x80000000,
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

enum devx_access_flags {
	DEVX_ACCESS_LOCAL_WRITE		= 1,
	DEVX_ACCESS_REMOTE_WRITE		= (1<<1),
	DEVX_ACCESS_REMOTE_READ		= (1<<2),
	DEVX_ACCESS_REMOTE_ATOMIC	= (1<<3),
	DEVX_ACCESS_MW_BIND		= (1<<4),
	DEVX_ACCESS_ZERO_BASED		= (1<<5),
	DEVX_ACCESS_ON_DEMAND		= (1<<6),
};

enum mlx5_cap_mode {
	HCA_CAP_OPMOD_GET_MAX	= 0,
	HCA_CAP_OPMOD_GET_CUR	= 1,
};

enum {
	MLX5_OPCODE_NOP			= 0x00,
	MLX5_OPCODE_SEND_INVAL		= 0x01,
	MLX5_OPCODE_RDMA_WRITE		= 0x08,
	MLX5_OPCODE_RDMA_WRITE_IMM	= 0x09,
	MLX5_OPCODE_SEND		= 0x0a,
	MLX5_OPCODE_SEND_IMM		= 0x0b,
	MLX5_OPCODE_TSO			= 0x0e,
	MLX5_OPCODE_RDMA_READ		= 0x10,
	MLX5_OPCODE_ATOMIC_CS		= 0x11,
	MLX5_OPCODE_ATOMIC_FA		= 0x12,
	MLX5_OPCODE_ATOMIC_MASKED_CS	= 0x14,
	MLX5_OPCODE_ATOMIC_MASKED_FA	= 0x15,
	MLX5_OPCODE_FMR			= 0x19,
	MLX5_OPCODE_LOCAL_INVAL		= 0x1b,
	MLX5_OPCODE_CONFIG_CMD		= 0x1f,
	MLX5_OPCODE_UMR			= 0x25,
	MLX5_OPCODE_TAG_MATCHING	= 0x28
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
	void *ctx;
	uint32_t ncqe; /* log2 of minimum number of entries required for CQ */
	uint32_t create_flags;
	uint32_t eqn;
	uint32_t uar;
};

struct mdev_tis_attr {
	void *ctx;
	uint32_t td;
};

struct mdev_eq_attr {
	void *ctx;
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
	void *ctx;
	uint32_t nelements;
	uint8_t rlkey;
	uint8_t fre;
	uint8_t inline_mode;
	uint32_t cqn;
	uint32_t tisn;
	uint32_t uar;
	uint32_t state;
	uint32_t max_inline_data;
	uint32_t max_tso_header;
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
	uint32_t index;
	void *uar;
};

struct mdev_mem_reg {
	uint32_t index;
	struct devx_obj_handle *object;
};

struct mdev_dbr{
	size_t offset;
	uint32_t index;
	void * addr;
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
	struct mlx5_txq_data *(*txqs)[]; /* TX queues. */
	unsigned int txqs_n; /* TX queues array size. */
	LIST_HEAD(mr, mlx5_mdev_mr) mr; /* Memory region. */
	LIST_HEAD(txq, mlx5_txq_ctrl) txqsctrl; /* DPDK Tx queues. */
	LIST_HEAD(txqmdev, mlx5_txq_mdev) txqsmdev; /* mdev Tx queues. */
	struct mlx5_mdev_cap caps;
	//struct mlx5_mdev_cap caps_max;  TODO: Do we need caps max ?
	uint32_t link_speed_capa; /* Link speed capabilities. */
	struct mlx5_xstats_ctrl xstats_ctrl; /* Extended stats control. */
};

struct mdev_cq {
	void *dev;
	const struct rte_memzone *buf;
	struct mdev_mem_reg mem;
	struct mdev_dbr dbrec;
	uint32_t cqe_size;
	uint32_t uar_page;
	uint32_t cqn;
	uint32_t cons_index;
	uint32_t eqn;
	uint32_t ncqe;
	struct devx_obj_handle *object;
};

struct mdev_eq {
	void *ctx;
	const struct rte_memzone *buf;
	struct mdev_mem_reg mem;
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
	void * dbr_addr;
	uint32_t hw_counter;
	uint32_t sw_counter;
	uint8_t stride_sz; /* The size of a WQ stride equals 2^log_wq_stride. */
	uint8_t page_sz; /* The size of a WQ stride equals 2^log_wq_stride. */
	uint8_t sz; /* The size of a WQ stride equals 2^log_wq_stride. */
	const struct rte_memzone *buf;
	struct mdev_mem_reg mem;
	struct mdev_dbr dbrec;

};


struct mdev_sq {
	void *ctx;
	uint32_t cqn;
	uint32_t tisn;
	uint32_t sqn;
	struct mdev_wq wq;
	struct devx_obj_handle *object;
};

struct mlx5_mdev_mkey {
	void *		obj;
	uint32_t	key;
};

struct mlx5_mdev_mkey_attr {
	uint64_t	addr;
	uint64_t	size;
	uint32_t	pas_id;
	uint32_t	pd;
};

int64_t mlx5_get_dbrec(struct mlx5_mdev_priv *priv);

/**
 * Lock private structure to protect it from concurrent access in the
 * control path.
 *
 * @param priv
 *   Pointer to private structure.
 */
static inline void
priv_lock(struct mlx5_mdev_priv *priv)
{
	rte_spinlock_lock(&priv->lock);
}

/**
 * Try to lock private structure to protect it from concurrent access in the
 * control path.
 *
 * @param priv
 *   Pointer to private structure.
 *
 * @return
 *   1 if the lock is successfully taken; 0 otherwise.
 */
static inline int
priv_trylock(struct mlx5_mdev_priv *priv)
{
	return rte_spinlock_trylock(&priv->lock);
}

/**
 * Unlock private structure.
 *
 * @param priv
 *   Pointer to private structure.
 */
static inline void
priv_unlock(struct mlx5_mdev_priv *priv)
{
	rte_spinlock_unlock(&priv->lock);
}

/* mlx5_prm_commands.c */

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
int
mlx5_mdev_query_sq(struct mdev_sq *sq,
		    struct mdev_sq_attr *sq_attr);
int
mlx5_mdev_modify_sq(struct mdev_sq *sq,
		    struct mdev_sq_attr *sq_attr);
int mlx5_mdev_alloc_pd(struct mlx5_mdev_priv *priv);
int mlx5_mdev_alloc_td(struct mlx5_mdev_priv *priv);
struct mlx5_mdev_mkey *mlx5_mdev_create_mkey(struct mlx5_mdev_priv *priv,
			struct mlx5_mdev_mkey_attr *mkey_attr);
int mlx5_mdev_query_hca_cap(void *ctx, struct mlx5_mdev_cap *caps);
int mlx5_mdev_check_hca_cap(struct mlx5_mdev_cap *caps);
int mlx5_mdev_query_vport_state(void *ctx,
				uint8_t *admin_state, uint8_t *state);



/* mlx5_ethdev.c */

struct mlx5_mdev_priv *mlx5_get_priv(struct rte_eth_dev *dev);
int priv_get_ifname(const struct mlx5_mdev_priv *, char (*)[IFNAMSIZ]);
int priv_ifreq(const struct mlx5_mdev_priv *, int req, struct ifreq *);
int priv_is_ib_cntr(const char *);
int priv_get_cntr_sysfs(struct mlx5_mdev_priv *, const char *, uint64_t *);
int priv_get_num_vfs(struct mlx5_mdev_priv *, uint16_t *);
int priv_get_mtu(struct mlx5_mdev_priv *, uint16_t *);
int priv_set_flags(struct mlx5_mdev_priv *, unsigned int, unsigned int);
eth_tx_burst_t
priv_select_tx_function(struct mlx5_mdev_priv *priv, struct rte_eth_dev *dev);
int mlx5_dev_configure(struct rte_eth_dev *dev);
int mlx5_mdev_device_to_pci_addr(const struct devx_device *device,
				 struct rte_pci_addr *pci_addr);
int mlx5_mdev_set_link_down(struct rte_eth_dev *dev);
int mlx5_mdev_set_link_up(struct rte_eth_dev *dev);
int mlx5_mdev_link_update(struct rte_eth_dev *dev, int wait_to_complete);


/*mlx5_mac.c */
int mdev_priv_get_mac(struct mlx5_mdev_priv *priv,
		      uint8_t (*mac)[ETHER_ADDR_LEN]);
void mlx5_mdev_mac_addr_remove(struct rte_eth_dev *dev, uint32_t index);
int mlx5_mdev_mac_addr_add(struct rte_eth_dev *dev, struct ether_addr *mac,
		  	   uint32_t index, uint32_t vmdq);
void mlx5_mdev_mac_addr_set(struct rte_eth_dev *dev,
			    struct ether_addr *mac_addr);

/* mlx5_mr.c */

struct mlx5_mdev_mr *priv_mr_new(struct mlx5_mdev_priv *, struct rte_mempool *);
struct mlx5_mdev_mr *priv_mr_get(struct mlx5_mdev_priv *, struct rte_mempool *);
int priv_mr_release(struct mlx5_mdev_priv *, struct mlx5_mdev_mr *);
int priv_mr_verify(struct mlx5_mdev_priv *);

/* mlx5_trigger.c */

int mlx5_mdev_start(struct rte_eth_dev *);
void mlx5_mdev_stop(struct rte_eth_dev *);
int priv_dev_traffic_enable(struct mlx5_mdev_priv *, struct rte_eth_dev *);
int priv_dev_traffic_disable(struct mlx5_mdev_priv *, struct rte_eth_dev *);
int priv_dev_traffic_restart(struct mlx5_mdev_priv *, struct rte_eth_dev *);
int mlx5_traffic_restart(struct rte_eth_dev *);

/* mlx5_mdev_rxtx.c */
uint16_t
mlx5_mdev_tx_burst(void *dpdk_txq, struct rte_mbuf **pkts, uint16_t pkts_n);


/* mlx5_stats.c */

void priv_xstats_init(struct mlx5_mdev_priv *);
int mlx5_mdev_stats_get(struct rte_eth_dev *, struct rte_eth_stats *);
void mlx5_mdev_stats_reset(struct rte_eth_dev *);
int mlx5_mdev_xstats_get(struct rte_eth_dev *,
		    struct rte_eth_xstat *, unsigned int);
void mlx5_mdev_xstats_reset(struct rte_eth_dev *);
int mlx5_mdev_xstats_get_names(struct rte_eth_dev *,
			  struct rte_eth_xstat_name *, unsigned int);

#endif

