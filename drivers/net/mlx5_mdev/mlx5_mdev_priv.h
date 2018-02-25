/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2015 Mellanox.
 */

#ifndef MLX5_MDEV_PRIV_H_
#define MLX5_MDEV_PRIV_H_
#include <sys/ioctl.h>
#include <net/if.h>

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

struct mlx5_mdev_priv {
	struct rte_eth_dev *edev;
	void	*base_addr;
	void *dev;
	struct mlx5_mdev_db_page *db_page;
	int32_t page_size;
	int32_t cache_line_size;
	int32_t pd;
	uint64_t *uar;
	uint32_t uar_index;
	struct devx_obj_handle *pd_object;
	rte_spinlock_t lock; /* Lock for control functions. */
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

int priv_get_mac(struct mlx5_mdev_priv *, uint8_t (*)[ETHER_ADDR_LEN]);
int
priv_ifreq(const struct mlx5_mdev_priv *priv, int req, struct ifreq *ifr);
#endif

