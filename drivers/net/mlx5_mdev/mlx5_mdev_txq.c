/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2015 6WIND S.A.
 * Copyright 2015 Mellanox.
 */

#include <stddef.h>
#include <assert.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/mman.h>

#include <rte_mbuf.h>
#include <rte_malloc.h>
#include <rte_ethdev_driver.h>
#include <rte_common.h>

#include "mlx5_mdev_utils.h"
#include "mlx5_mdev_defs.h"
#include "mlx5_mdev.h"
#include "mlx5_mdev_rxtx.h"
#include "mlx5_mdev_priv.h"
//#include "mlx5_mdev_autoconf.h"
//#include "mlx5_mdev_glue.h"

/**
 * Allocate TX queue elements.
 *
 * @param txq_ctrl
 *   Pointer to TX queue structure.
 */
void
txq_alloc_elts(struct mlx5_txq_ctrl *txq_ctrl)
{
	const unsigned int elts_n = 1 << txq_ctrl->txq.elts_n;
	unsigned int i;

	for (i = 0; (i != elts_n); ++i)
		(*txq_ctrl->txq.elts)[i] = NULL;
	DEBUG("%p: allocated and configured %u WRs", (void *)txq_ctrl, elts_n);
	txq_ctrl->txq.elts_head = 0;
	txq_ctrl->txq.elts_tail = 0;
	txq_ctrl->txq.elts_comp = 0;
}

/**
 * Free TX queue elements.
 *
 * @param txq_ctrl
 *   Pointer to TX queue structure.
 */
static void
txq_free_elts(struct mlx5_txq_ctrl *txq_ctrl)
{
	const uint16_t elts_n = 1 << txq_ctrl->txq.elts_n;
	const uint16_t elts_m = elts_n - 1;
	uint16_t elts_head = txq_ctrl->txq.elts_head;
	uint16_t elts_tail = txq_ctrl->txq.elts_tail;
	struct rte_mbuf *(*elts)[elts_n] = txq_ctrl->txq.elts;

	DEBUG("%p: freeing WRs", (void *)txq_ctrl);
	txq_ctrl->txq.elts_head = 0;
	txq_ctrl->txq.elts_tail = 0;
	txq_ctrl->txq.elts_comp = 0;

	while (elts_tail != elts_head) {
		struct rte_mbuf *elt = (*elts)[elts_tail & elts_m];

		assert(elt != NULL);
		rte_pktmbuf_free_seg(elt);
#ifndef NDEBUG
		/* Poisoning. */
		memset(&(*elts)[elts_tail & elts_m],
		       0x77,
		       sizeof((*elts)[elts_tail & elts_m]));
#endif
		++elts_tail;
	}
}

/**
 * Returns the per-port supported offloads.
 *
 * @param priv
 *   Pointer to private structure.
 *
 * @return
 *   Supported Tx offloads.
 */
uint64_t
mlx5_priv_get_tx_port_offloads(struct mlx5_mdev_priv *priv)
{
	uint64_t offloads = (DEV_TX_OFFLOAD_MULTI_SEGS |
			     DEV_TX_OFFLOAD_VLAN_INSERT);
	struct mlx5_mdev_dev_config *config = &priv->config;

	if (config->hw_csum)
		offloads |= (DEV_TX_OFFLOAD_IPV4_CKSUM |
			     DEV_TX_OFFLOAD_UDP_CKSUM |
			     DEV_TX_OFFLOAD_TCP_CKSUM);
	if (config->tso)
		offloads |= DEV_TX_OFFLOAD_TCP_TSO;
	if (config->tunnel_en) {
		if (config->hw_csum)
			offloads |= DEV_TX_OFFLOAD_OUTER_IPV4_CKSUM;
		if (config->tso)
			offloads |= (DEV_TX_OFFLOAD_VXLAN_TNL_TSO |
				     DEV_TX_OFFLOAD_GRE_TNL_TSO);
	}
	return offloads;
}

/**
 * Checks if the per-queue offload configuration is valid.
 *
 * @param priv
 *   Pointer to private structure.
 * @param offloads
 *   Per-queue offloads configuration.
 *
 * @return
 *   1 if the configuration is valid, 0 otherwise.
 */
static int
priv_is_tx_queue_offloads_allowed(struct mlx5_mdev_priv *priv, uint64_t offloads)
{
	uint64_t port_offloads = priv->dev->data->dev_conf.txmode.offloads;
	uint64_t port_supp_offloads = mlx5_priv_get_tx_port_offloads(priv);

	/* There are no Tx offloads which are per queue. */
	if ((offloads & port_supp_offloads) != offloads)
		return 0;
	if ((port_offloads ^ offloads) & port_supp_offloads)
		return 0;
	return 1;
}

/**
 * DPDK callback to configure a TX queue.
 *
 * @param dev
 *   Pointer to Ethernet device structure.
 * @param idx
 *   TX queue index.
 * @param desc
 *   Number of descriptors to configure in queue.
 * @param socket
 *   NUMA socket on which memory must be allocated.
 * @param[in] conf
 *   Thresholds parameters.
 *
 * @return
 *   0 on success, negative errno value on failure.
 */
int
mlx5_tx_queue_setup(struct rte_eth_dev *dev, uint16_t idx, uint16_t desc,
		    unsigned int socket, const struct rte_eth_txconf *conf)
{
	struct mlx5_mdev_priv *priv = dev->data->dev_private;
	struct mlx5_txq_data *txq = (*priv->txqs)[idx];
	struct mlx5_txq_ctrl *txq_ctrl =
		container_of(txq, struct mlx5_txq_ctrl, txq);
	int ret = 0;

	priv_lock(priv);
	/*
	 * Don't verify port offloads for application which
	 * use the old API.
	 */
	if (!!(conf->txq_flags & ETH_TXQ_FLAGS_IGNORE) &&
	    !priv_is_tx_queue_offloads_allowed(priv, conf->offloads)) {
		ret = ENOTSUP;
		ERROR("%p: Tx queue offloads 0x%" PRIx64 " don't match port "
		      "offloads 0x%" PRIx64 " or supported offloads 0x%" PRIx64,
		      (void *)dev, conf->offloads,
		      dev->data->dev_conf.txmode.offloads,
		      mlx5_priv_get_tx_port_offloads(priv));
		goto out;
	}
	if (desc <= MLX5_TX_COMP_THRESH) {
		WARN("%p: number of descriptors requested for TX queue %u"
		     " must be higher than MLX5_TX_COMP_THRESH, using"
		     " %u instead of %u",
		     (void *)dev, idx, MLX5_TX_COMP_THRESH + 1, desc);
		desc = MLX5_TX_COMP_THRESH + 1;
	}
	if (!rte_is_power_of_2(desc)) {
		desc = 1 << log2above(desc);
		WARN("%p: increased number of descriptors in TX queue %u"
		     " to the next power of two (%d)",
		     (void *)dev, idx, desc);
	}
	DEBUG("%p: configuring queue %u for %u descriptors",
	      (void *)dev, idx, desc);
	if (idx >= priv->txqs_n) {
		ERROR("%p: queue index out of range (%u >= %u)",
		      (void *)dev, idx, priv->txqs_n);
		priv_unlock(priv);
		return -EOVERFLOW;
	}
	if (!mlx5_priv_txq_releasable(priv, idx)) {
		ret = EBUSY;
		ERROR("%p: unable to release queue index %u",
		      (void *)dev, idx);
		goto out;
	}
	mlx5_priv_txq_release(priv, idx);
	txq_ctrl = mlx5_priv_txq_new(priv, idx, desc, socket, conf);
	if (!txq_ctrl) {
		ERROR("%p: unable to allocate queue index %u",
		      (void *)dev, idx);
		ret = ENOMEM;
		goto out;
	}
	DEBUG("%p: adding TX queue %p to list",
	      (void *)dev, (void *)txq_ctrl);
	(*priv->txqs)[idx] = &txq_ctrl->txq;
out:
	priv_unlock(priv);
	return -ret;
}

/**
 * DPDK callback to release a TX queue.
 *
 * @param dpdk_txq
 *   Generic TX queue pointer.
 */
void
mlx5_tx_queue_release(void *dpdk_txq)
{
	struct mlx5_txq_data *txq = (struct mlx5_txq_data *)dpdk_txq;
	struct mlx5_txq_ctrl *txq_ctrl;
	struct mlx5_mdev_priv *priv;
	unsigned int i;

	if (txq == NULL)
		return;
	txq_ctrl = container_of(txq, struct mlx5_txq_ctrl, txq);
	priv = txq_ctrl->priv;
	priv_lock(priv);
	for (i = 0; (i != priv->txqs_n); ++i)
		if ((*priv->txqs)[i] == txq) {
			DEBUG("%p: removing TX queue %p from list",
			      (void *)priv->dev, (void *)txq_ctrl);
			mlx5_priv_txq_release(priv, i);
			break;
		}
	priv_unlock(priv);
}

#if 0
/**
 * Mmap TX UAR(HW doorbell) pages into reserved UAR address space.
 * Both primary and secondary process do mmap to make UAR address
 * aligned.
 *
 * @param[in] priv
 *   Pointer to private structure.
 * @param fd
 *   Verbs file descriptor to map UAR pages.
 *
 * @return
 *   0 on success, errno value on failure.
 */
int
priv_tx_uar_remap(struct mlx5_mdev_priv *priv, int fd)
{
	unsigned int i, j;
	uintptr_t pages[priv->txqs_n];
	unsigned int pages_n = 0;
	uintptr_t uar_va;
	uintptr_t off;
	void *addr;
	void *ret;
	struct mlx5_txq_data *txq;
	struct mlx5_txq_ctrl *txq_ctrl;
	int already_mapped;
	size_t page_size = sysconf(_SC_PAGESIZE);
	int r;

	memset(pages, 0, priv->txqs_n * sizeof(uintptr_t));
	/*
	 * As rdma-core, UARs are mapped in size of OS page size.
	 * Use aligned address to avoid duplicate mmap.
	 * Ref to libmlx5 function: mlx5_init_context()
	 */
	for (i = 0; i != priv->txqs_n; ++i) {
		if (!(*priv->txqs)[i])
			continue;
		txq = (*priv->txqs)[i];
		txq_ctrl = container_of(txq, struct mlx5_txq_ctrl, txq);
		/* UAR addr form verbs used to find dup and offset in page. */
		uar_va = (uintptr_t)txq_ctrl->bf_reg_orig;
		off = uar_va & (page_size - 1); /* offset in page. */
		uar_va = RTE_ALIGN_FLOOR(uar_va, page_size); /* page addr. */
		already_mapped = 0;
		for (j = 0; j != pages_n; ++j) {
			if (pages[j] == uar_va) {
				already_mapped = 1;
				break;
			}
		}
		/* new address in reserved UAR address space. */
		addr = RTE_PTR_ADD(priv->uar_base,
				   uar_va & (MLX5_UAR_SIZE - 1));
		if (!already_mapped) {
			pages[pages_n++] = uar_va;
			/* fixed mmap to specified address in reserved
			 * address space.
			 */
			ret = mmap(, page_size,
				   PROT_WRITE, MAP_FIXED | MAP_SHARED, fd,
				   txq_ctrl->uar_mmap_offset);
			if (ret != addr) {
				/* fixed mmap have to return same address */
				ERROR("call to mmap failed on UAR for txq %d\n",
				      i);
				r = ENXIO;
				return r;
			}
		}
		if (rte_eal_process_type() == RTE_PROC_PRIMARY) /* save once */
			txq_ctrl->txq.bf_reg = RTE_PTR_ADD((void *)addr, off);
		else
			assert(txq_ctrl->txq.bf_reg ==
			       RTE_PTR_ADD((void *)addr, off));
	}
	return 0;
}
#endif
/**
 * Check if the burst function is using eMPW.
 *
 * @param tx_pkt_burst
 *   Tx burst function pointer.
 *
 * @return
 *   1 if the burst function is using eMPW, 0 otherwise.
 */
static int
is_empw_burst_func(eth_tx_burst_t __rte_unused tx_pkt_burst)
{
	//FIXME
//	if (tx_pkt_burst == mlx5_tx_burst_raw_vec ||
//	    tx_pkt_burst == mlx5_tx_burst_vec ||
//	    tx_pkt_burst == mlx5_tx_burst_empw)
//		return 1;
	return 0;
}

/**
 * Create the Tx queue Verbs object.
 *
 * @param priv
 *   Pointer to private structure.
 * @param idx
 *   Queue index in DPDK Rx queue array
 *
 * @return
 *   The Verbs object initialised if it can be created.
 */
struct mlx5_txq_mdev*
mlx5_priv_txq_mdev_new(struct mlx5_mdev_priv *priv, uint16_t idx)
{

	struct mlx5_txq_data *txq_data = (*priv->txqs)[idx];
	struct mlx5_txq_ctrl *txq_ctrl =
		container_of(txq_data, struct mlx5_txq_ctrl, txq);
	struct mlx5_txq_mdev tmpl;
	struct mlx5_txq_mdev *txq_mdev;
	union {
		struct mdev_eq_attr meq_attr;
		struct mdev_cq_attr mcq_attr;
		struct mdev_tis_attr tis_attr;
		struct mdev_sq_attr sq_attr;
	} attr;
	unsigned int cqe_n;
	struct mdev_eq *meq = NULL;
	struct mdev_cq *mcq = NULL;
	struct mdev_tis *mtis = NULL;
	struct mdev_sq *msq = NULL;
#if 0
	struct mlx5dv_qp qp = { .comp_mask = MLX5DV_QP_MASK_UAR_MMAP_OFFSET };
	struct mlx5dv_cq cq_info;
	struct mlx5dv_obj obj;
#endif
	const int desc = 1 << txq_data->elts_n;
	eth_tx_burst_t tx_pkt_burst = priv_select_tx_function(priv, priv->dev);
#if 0
	int ret = 0;

	assert(txq_data);
	priv->verbs_alloc_ctx.type = MLX5_VERBS_ALLOC_TYPE_TX_QUEUE;
	priv->verbs_alloc_ctx.obj = txq_ctrl;
	if (mlx5_getenv_int("MLX5_ENABLE_CQE_COMPRESSION")) {
		ERROR("MLX5_ENABLE_CQE_COMPRESSION must never be set");
		goto error;
	}
#endif
	memset(&tmpl, 0, sizeof(struct mlx5_txq_mdev));

	/* MRs will be registered in mp2mr[] later. */

	cqe_n = ((desc / MLX5_TX_COMP_THRESH) - 1) ?
		((desc / MLX5_TX_COMP_THRESH) - 1) : 1;
	if (is_empw_burst_func(tx_pkt_burst))
		cqe_n += MLX5_TX_COMP_THRESH_INLINE_DIV;
	attr.meq_attr.ctx = priv->ctx;
	attr.meq_attr.eqe = cqe_n;
	attr.meq_attr.uar = priv->uar.index;
	meq = mlx5_mdev_create_eq(priv, &attr.meq_attr);
	if (meq == NULL) {
		ERROR("%p: EQ creation failure", (void *)txq_ctrl);
//		goto error;
	}

	attr.mcq_attr.ncqe = log2above(cqe_n);
	attr.mcq_attr.eqn = meq->eqn;
	attr.mcq_attr.ctx = priv->ctx;
	attr.mcq_attr.uar = priv->uar.index;
	attr.mcq_attr.create_flags = 0;
	//tmpl.cq = mlx5_glue->create_cq(priv->ctx, cqe_n, NULL, NULL, 0);
	mcq = mlx5_mdev_create_cq(priv, &attr.mcq_attr);
	if (mcq == NULL) {
		ERROR("%p: CQ creation failure", (void *)txq_ctrl);
//		goto error;
	}

	attr.tis_attr.ctx = priv->ctx;
	attr.tis_attr.td = priv->td.td;
	mtis = mlx5_mdev_create_tis(priv, &attr.tis_attr);
	if (mtis == NULL) {
		ERROR("%p: TIS creation failure", (void *)txq_ctrl);
//		goto error;
	}

	attr.sq_attr.ctx = priv->ctx;
	attr.sq_attr.cqn = mcq->cqn;
	attr.sq_attr.tisn = mtis->tisn;
	attr.sq_attr.fre = 0; // TODO: get real value
	attr.sq_attr.nelements = desc;//FIXME should be based on device attributes
	attr.sq_attr.inline_mode = 0; // TODO: take from mlx5_ifc_nic_vport_context_bits
	attr.sq_attr.rlkey = 0;
	attr.sq_attr.wq.pd = priv->pd.pd;
	attr.sq_attr.uar = priv->uar.index;
	msq = mlx5_mdev_create_sq(priv, &attr.sq_attr);
	
	mlx5_mdev_query_sq(msq, &attr.sq_attr);
	mlx5_mdev_modify_sq(msq, &attr.sq_attr);
	mlx5_mdev_query_sq(msq, &attr.sq_attr);
	if (msq == NULL) {
		ERROR("%p: msq creation failure", (void *)txq_ctrl);
//		goto error;
	}

	txq_mdev = rte_calloc_socket(__func__, 1, sizeof(struct mlx5_txq_mdev), 0,
					    txq_ctrl->socket);
	if (!txq_mdev) {
		ERROR("%p: cannot allocate memory", (void *)txq_ctrl);
//		goto error;
	}

//	mlx5 code
//	txq_data->cqe_n = log2above(cq_info.cqe_cnt);
//		txq_data->qp_num_8s = tmpl.qp->qp_num << 8;
//		txq_data->wqes = qp.sq.buf;
//		txq_data->wqe_n = log2above(qp.sq.wqe_cn`t);
//		txq_data->qp_db = &qp.dbrec[MLX5_SND_DBR];
//		txq_ctrl->bf_reg_orig = qp.bf.reg;
//		txq_data->cq_db = cq_info.dbrec;


	txq_data->cqe_n = mcq->ncqe;
	txq_data->sq_num_8s = msq->sqn << 8;
	txq_data->wqes = msq->wq.buf->addr; //buf;
	//FIXME should be check if number of elements in cq equal to num of elements in wq
	txq_data->wqe_n = log2above(desc);//FIXME should be based on returned value from create sq
	txq_data->qp_db = (uint32_t *)(msq->wq.dbrec.addr);
	txq_data->cq_db = (uint32_t *)(mcq->dbrec.addr);
//	txq_ctrl->bf_reg_orig = (void *)((uint8_t *)priv->uar.uar + 0x800);
	txq_ctrl->txq.bf_reg = (void *)((uint8_t *)priv->uar.uar + 0x800);
	txq_data->cqes =
		(volatile struct mlx5_cqe (*)[])
		(uintptr_t)mcq->buf->addr;
	txq_data->cq_ci = 0;
	#ifndef NDEBUG
		txq_data->cq_pi = 0;
	#endif

	txq_data->wqe_ci = 0;
	txq_data->wqe_pi = 0;
	txq_mdev->sq = msq;
	txq_mdev->cq = mcq;
	txq_mdev->eq = meq;
	txq_mdev->tis = mtis;
	rte_atomic32_inc(&txq_mdev->refcnt);
	LIST_INSERT_HEAD(&priv->txqsmdev, txq_mdev, next);
	return txq_mdev;
#if 0 //~~~~

#if 0
	// TODO where to put those vals ?

		.cap = {
			/* Max number of outstanding WRs. */
			.max_send_wr =
				((priv->device_attr.orig_attr.max_qp_wr <
				  desc) ?
				 priv->device_attr.orig_attr.max_qp_wr :
				 desc),
			/*
			 * Max number of scatter/gather elements in a WR,
			 * must be 1 to prevent libmlx5 from trying to affect
			 * too much memory. TX gather is not impacted by the
			 * priv->device_attr.max_sge limit and will still work
			 * properly.
			 */
			.max_send_sge = 1,
		},
		.qp_type = IBV_QPT_RAW_PACKET,
		/*
		 * Do *NOT* enable this, completions events are managed per
		 * Tx burst.
		 */
	if (txq_data->max_inline)
		attr.init.cap.max_inline_data = txq_ctrl->max_inline_data;
	if (txq_data->tso_en) {
		attr.init.max_tso_header = txq_ctrl->max_tso_header;
		attr.init.comp_mask |= IBV_QP_INIT_ATTR_MAX_TSO_HEADER;
	}
	tmpl.qp = mlx5_glue->create_qp_ex(priv->ctx, &attr.init);
	if (tmpl.qp == NULL) {
		ERROR("%p: QP creation failure", (void *)txq_ctrl);
		goto error;
	}
#endif
	if(msq == NULL) {
		ERROR("%p: SQ creation failure", (void *)txq_ctrl);
		goto error;
	}

#if 0
	attr.sq_attr = (struct mdev_sq_attr){
		/* Move the QP to this state. */
		.sq_state = IBV_QPS_INIT,
		/* Primary port number. */
		.port_num = priv->port
	};
	ret = mlx5_mdev_modify_sq(msq, &attr.sq_attr, IBV_QP_STATE | IBV_QP_POR); //TODO: what to do here ?

	ret = mlx5_glue->modify_qp(tmpl.qp, &attr.mod,
				   (IBV_QP_STATE | IBV_QP_PORT));
	if (ret) {
		ERROR("%p: QP state to IBV_QPS_INIT failed", (void *)txq_ctrl);
		goto error;
	}
	attr.mod = (struct ibv_qp_attr){
		.qp_state = IBV_QPS_RTR
	};
	ret = mlx5_glue->modify_qp(tmpl.qp, &attr.mod, IBV_QP_STATE);
	if (ret) {
		ERROR("%p: QP state to IBV_QPS_RTR failed", (void *)txq_ctrl);
		goto error;
	}
	attr.mod.qp_state = IBV_QPS_RTS;
	ret = mlx5_glue->modify_qp(tmpl.qp, &attr.mod, IBV_QP_STATE);
	if (ret) {
		ERROR("%p: QP state to IBV_QPS_RTS failed", (void *)txq_ctrl);
		goto error;
	}
#endif
	txq_mdev = rte_calloc_socket(__func__, 1, sizeof(struct mlx5_txq_mdev), 0,
				    txq_ctrl->socket);
	if (!txq_mdev) {
		ERROR("%p: cannot allocate memory", (void *)txq_ctrl);
		goto error;
	}
#if 0
	obj.cq.in = tmpl.cq;
	obj.cq.out = &cq_info;
	obj.qp.in = tmpl.qp;
	obj.qp.out = &qp;
	ret = mlx5_glue->dv_init_obj(&obj, MLX5DV_OBJ_CQ | MLX5DV_OBJ_QP);
	if (ret != 0)
		goto error;
	if (cq_info.cqe_size != RTE_CACHE_LINE_SIZE) {
		ERROR("Wrong MLX5_CQE_SIZE environment variable value: "
		      "it should be set to %u", RTE_CACHE_LINE_SIZE);
		goto error;
	}
#endif
	if (mcq->cqe_size != RTE_CACHE_LINE_SIZE) {
		ERROR("Wrong MLX5_CQE_SIZE environment variable value: "
		      "it should be set to %u", RTE_CACHE_LINE_SIZE);
		goto error;
	}

	txq_data->cqe_n = mcq->ncqe;
	txq_data->sq_num_8s = msq->sqn << 8;
	txq_data->wqes = msq->wq.buf->addr; //buf;
	txq_data->wqe_n = msq->wq.wqe_cnt;
	txq_data->qp_db = &(msq->wq.dbr_addr[MLX5_SND_DBR]);
	/// txq_ctrl->bf_reg_orig = wq.qp.bf.reg;
	txq_data->cq_db = mcq->dbr_addr; // cq_info.dbrec;

	txq_data->cqes =
		(volatile struct mlx5_cqe (*)[])
		(uintptr_t)mcq->buf->addr;
	txq_data->cq_ci = 0;
#ifndef NDEBUG
	txq_data->cq_pi = 0;
#endif
	txq_data->wqe_ci = 0;
	txq_data->wqe_pi = 0;
	txq_mdev->sq = msq;
	txq_mdev->cq = mcq;
	txq_mdev->eq = meq;
	txq_mdev->tis = mtis;
	rte_atomic32_inc(&txq_mdev->refcnt);
#if 0
	// TODO: Is this ctx->uar ?
	if (qp.comp_mask & MLX5DV_QP_MASK_UAR_MMAP_OFFSET) {
		txq_ctrl->uar_mmap_offset = qp.uar_mmap_offset;
	} else {
		ERROR("Failed to retrieve UAR info, invalid libmlx5.so version");
		goto error;
	}
	DEBUG("%p: Verbs Tx queue %p: refcnt %d", (void *)priv,
	      (void *)txq_ibv, rte_atomic32_read(&txq_ibv->refcnt));
#endif
	LIST_INSERT_HEAD(&priv->txqsmdev, txq_mdev, next);
	priv->verbs_alloc_ctx.type = MLX5_VERBS_ALLOC_TYPE_NONE;
	ERROR("================= Return success (sqn = %d)", msq->sqn);

	ERROR("========= Testing RX (in Tx path) ===========");
	{
#define MOTIH_NRXQS 4
		int i;
		struct mdev_rqt *rqt;
		struct mdev_tir *tir;
		struct mdev_rqt_attr rqt_attr;
		struct mdev_tir_attr tir_attr;
		uint32_t rqn_arr[MOTIH_NRXQS];
		struct mdev_rq_attr rq_attr;
		struct mdev_rq *rq[MOTIH_NRXQS];
		rq_attr.ctx = priv->mpriv.dev_context;
		rq_attr.cqn = mcq->cqn;
		rq_attr.csid = 0;
		rq_attr.nelements = cqe_n;
		rq_attr.rlkey = 0;
		rq_attr.vsd = 0;
		rq_attr.wq.pd = priv->mpriv.dev_context->pd;
		for (i = 0; i < MOTIH_NRXQS; i++) {
			rq[i] = mlx5_mdev_create_rq(&priv->mpriv, &rq_attr);
			if ((void *)rq[i] == NULL) {
				ERROR("mlx5_mdev_create_rq FAILED");
				goto error;
			} else {
				rqn_arr[i] = rq[i]->rqn;
				ERROR("======= rqn  %d ======", rq[i]->rqn);
			}
		}
		rqt_attr.log_num_rq = log2above(MOTIH_NRXQS);
		rqt_attr.rqn = rqn_arr;
		rqt = mlx5_mdev_create_rqt(&priv->mpriv, &rqt_attr);
		if ((void *)rqt == NULL) {
			ERROR("mlx5_mdev_create_rqt FAILED");
			goto error;
		} else {
			ERROR("======= rqtn  %d ======", rqt->rqtn);
		}
		tir_attr.disp_type = MLX5_TIR_DISP_DIRECT;
		tir_attr.direct.rqn = rqn_arr[0];
		tir_attr.td = priv->mpriv.dev_context->td;
		tir = mlx5_mdev_create_tir(&priv->mpriv, &tir_attr);
		if ((void *)tir == NULL) {
			ERROR("mlx5_mdev_create_tir FAILED");
		} else {
			ERROR("======= tirn  %d ======", tir->tirn);
		}
	}
	return txq_mdev;
error:
	if (msq)
		claim_zero(mlx5_mdev_destroy_sq(msq));
	if (mcq)
		claim_zero(mlx5_mdev_destroy_cq(mcq));
	if(mtis)
		claim_zero(mlx5_mdev_destroy_tis(mtis));
	priv->verbs_alloc_ctx.type = MLX5_VERBS_ALLOC_TYPE_NONE;

	return NULL;
#endif
}

/**
 * Get an Tx queue Verbs object.
 *
 * @param priv
 *   Pointer to private structure.
 * @param idx
 *   Queue index in DPDK Rx queue array
 *
 * @return
 *   The Verbs object if it exists.
 */
struct mlx5_txq_mdev*
mlx5_priv_txq_mdev_get(struct mlx5_mdev_priv *priv, uint16_t idx)
{
	struct mlx5_txq_ctrl *txq_ctrl;

	if (idx >= priv->txqs_n)
		return NULL;
	if (!(*priv->txqs)[idx])
		return NULL;
	txq_ctrl = container_of((*priv->txqs)[idx], struct mlx5_txq_ctrl, txq);
	if (txq_ctrl->mdev) {
		rte_atomic32_inc(&txq_ctrl->mdev->refcnt);
		DEBUG("%p: Verbs Tx queue %p: refcnt %d", (void *)priv,
		      (void *)txq_ctrl->mdev,
		      rte_atomic32_read(&txq_ctrl->mdev->refcnt));
	}
	return txq_ctrl->mdev;
}

/**
 * Release an Tx verbs queue object.
 *
 * @param priv
 *   Pointer to private structure.
 * @param txq_ibv
 *   Verbs Tx queue object.
 *
 * @return
 *   0 on success, errno on failure.
 */
int
mlx5_priv_txq_mdev_release(struct mlx5_mdev_priv __rte_unused *priv, struct mlx5_txq_mdev __rte_unused *txq_mdev)
{
#if 0
	(void)priv;
	assert(txq_mdev);
	DEBUG("%p: Verbs Tx queue %p: refcnt %d", (void *)priv,
	      (void *)txq_mdev, rte_atomic32_read(&txq_mdev->refcnt));
	if (rte_atomic32_dec_and_test(&txq_mdev->refcnt)) {
		claim_zero(mlx5_mdev_destroy_sq(txq_mdev->sq));
		claim_zero(mlx5_mdev_destroy_cq(txq_mdev->cq));
		claim_zero(mlx5_mdev_destroy_tis(txq_mdev->tis));
		claim_zero(mlx5_mdev_destroy_eq(txq_mdev->eq));
		LIST_REMOVE(txq_mdev, next);
		rte_free(txq_mdev);
		return 0;
	}
#endif
	return EBUSY;
}

/**
 * Return true if a single reference exists on the object.
 *
 * @param priv
 *   Pointer to private structure.
 * @param txq_ibv
 *   Verbs Tx queue object.
 */
int
mlx5_priv_txq_mdev_releasable(struct mlx5_mdev_priv *priv, struct mlx5_txq_mdev *txq_mdev)
{
	(void)priv;
	assert(txq_mdev);
	return (rte_atomic32_read(&txq_mdev->refcnt) == 1);
}

/**
 * Verify the Verbs Tx queue list is empty
 *
 * @param priv
 *  Pointer to private structure.
 *
 * @return the number of object not released.
 */
int
mlx5_priv_txq_mdev_verify(struct mlx5_mdev_priv *priv)
{
	int ret = 0;
	struct mlx5_txq_mdev *txq_mdev;

	LIST_FOREACH(txq_mdev, &priv->txqsmdev, next) {
		DEBUG("%p: mdev Tx queue %p still referenced", (void *)priv,
		      (void *)txq_mdev);
		++ret;
	}
	return ret;
}

/**
 * Set Tx queue parameters from device configuration.
 *
 * @param txq_ctrl
 *   Pointer to Tx queue control structure.
 */
static void
txq_set_params(struct mlx5_txq_ctrl *txq_ctrl)
{
	struct mlx5_mdev_priv *priv = txq_ctrl->priv;
	struct mlx5_mdev_dev_config *config = &priv->config;
	const unsigned int max_tso_inline =
		((MLX5_MAX_TSO_HEADER + (RTE_CACHE_LINE_SIZE - 1)) /
		 RTE_CACHE_LINE_SIZE);
	unsigned int txq_inline;
	unsigned int txqs_inline;
	unsigned int inline_max_packet_sz;
	eth_tx_burst_t tx_pkt_burst = priv_select_tx_function(priv, priv->dev);
	int is_empw_func = is_empw_burst_func(tx_pkt_burst);
	int tso = !!(txq_ctrl->txq.offloads & DEV_TX_OFFLOAD_TCP_TSO);

	txq_inline = (config->txq_inline == MLX5_ARG_UNSET) ?
		0 : config->txq_inline;
	txqs_inline = (config->txqs_inline == MLX5_ARG_UNSET) ?
		0 : config->txqs_inline;
	inline_max_packet_sz =
		(config->inline_max_packet_sz == MLX5_ARG_UNSET) ?
		0 : config->inline_max_packet_sz;
	if (is_empw_func) {
		if (config->txq_inline == MLX5_ARG_UNSET)
			txq_inline = MLX5_WQE_SIZE_MAX - MLX5_WQE_SIZE;
		if (config->txqs_inline == MLX5_ARG_UNSET)
			txqs_inline = MLX5_EMPW_MIN_TXQS;
		if (config->inline_max_packet_sz == MLX5_ARG_UNSET)
			inline_max_packet_sz = MLX5_EMPW_MAX_INLINE_LEN;
		txq_ctrl->txq.mpw_hdr_dseg = config->mpw_hdr_dseg;
		txq_ctrl->txq.inline_max_packet_sz = inline_max_packet_sz;
	}
	if (txq_inline && priv->txqs_n >= txqs_inline) {
		unsigned int ds_cnt;

		txq_ctrl->txq.max_inline =
			((txq_inline + (RTE_CACHE_LINE_SIZE - 1)) /
			 RTE_CACHE_LINE_SIZE);
		if (is_empw_func) {
			/* To minimize the size of data set, avoid requesting
			 * too large WQ.
			 */
			txq_ctrl->max_inline_data =
				((RTE_MIN(txq_inline,
					  inline_max_packet_sz) +
				  (RTE_CACHE_LINE_SIZE - 1)) /
				 RTE_CACHE_LINE_SIZE) * RTE_CACHE_LINE_SIZE;
		} else if (tso) {
			int inline_diff = txq_ctrl->txq.max_inline -
					  max_tso_inline;

			/*
			 * Adjust inline value as Verbs aggregates
			 * tso_inline and txq_inline fields.
			 */
			txq_ctrl->max_inline_data = inline_diff > 0 ?
					       inline_diff *
					       RTE_CACHE_LINE_SIZE :
					       0;
		} else {
			txq_ctrl->max_inline_data =
				txq_ctrl->txq.max_inline * RTE_CACHE_LINE_SIZE;
		}
		/*
		 * Check if the inline size is too large in a way which
		 * can make the WQE DS to overflow.
		 * Considering in calculation:
		 *      WQE CTRL (1 DS)
		 *      WQE ETH  (1 DS)
		 *      Inline part (N DS)
		 */
		ds_cnt = 2 + (txq_ctrl->txq.max_inline / MLX5_WQE_DWORD_SIZE);
		if (ds_cnt > MLX5_DSEG_MAX) {
			unsigned int max_inline = (MLX5_DSEG_MAX - 2) *
						  MLX5_WQE_DWORD_SIZE;

			max_inline = max_inline - (max_inline %
						   RTE_CACHE_LINE_SIZE);
			WARN("txq inline is too large (%d) setting it to "
			     "the maximum possible: %d\n",
			     txq_inline, max_inline);
			txq_ctrl->txq.max_inline = max_inline /
						   RTE_CACHE_LINE_SIZE;
		}
	}
	if (tso) {
		txq_ctrl->max_tso_header = max_tso_inline * RTE_CACHE_LINE_SIZE;
		txq_ctrl->txq.max_inline = RTE_MAX(txq_ctrl->txq.max_inline,
						   max_tso_inline);
		txq_ctrl->txq.tso_en = 1;
	}
	txq_ctrl->txq.tunnel_en = config->tunnel_en;
	ERROR("==== MOTIH %s: end", __FUNCTION__);
}

/**
 * Create a DPDK Tx queue.
 *
 * @param priv
 *   Pointer to private structure.
 * @param idx
 *   TX queue index.
 * @param desc
 *   Number of descriptors to configure in queue.
 * @param socket
 *   NUMA socket on which memory must be allocated.
 * @param[in] conf
 *  Thresholds parameters.
 *
 * @return
 *   A DPDK queue object on success.
 */
struct mlx5_txq_ctrl*
mlx5_priv_txq_new(struct mlx5_mdev_priv *priv, uint16_t idx, uint16_t desc,
		  unsigned int socket,
		  const struct rte_eth_txconf *conf)
{
	struct mlx5_txq_ctrl *tmpl;
	tmpl = rte_calloc_socket("TXQ", 1,
				 sizeof(*tmpl) +
				 desc * sizeof(struct rte_mbuf *),
				 0, socket);
	if (!tmpl)
		return NULL;
	assert(desc > MLX5_TX_COMP_THRESH);
	tmpl->txq.offloads = conf->offloads;
	tmpl->priv = priv;
	tmpl->socket = socket;
	tmpl->txq.elts_n = log2above(desc);
	ERROR("============ tmpl->txq.elts_n %d (desc %d)", tmpl->txq.elts_n, desc);
	txq_set_params(tmpl);
	/* MRs will be registered in mp2mr[] later. */
	DEBUG("priv->device_attr.max_qp_wr is %d",
	      priv->device_attr.orig_attr.max_qp_wr);
	DEBUG("priv->device_attr.max_sge is %d",
	      priv->device_attr.orig_attr.max_sge);
	tmpl->txq.elts =
		(struct rte_mbuf *(*)[1 << tmpl->txq.elts_n])(tmpl + 1);
	tmpl->txq.stats.idx = idx;
	rte_atomic32_inc(&tmpl->refcnt);
	DEBUG("%p: Tx queue %p: refcnt %d", (void *)priv,
	      (void *)tmpl, rte_atomic32_read(&tmpl->refcnt));
	LIST_INSERT_HEAD(&priv->txqsctrl, tmpl, next);
	return tmpl;
}

/**
 * Get a Tx queue.
 *
 * @param priv
 *   Pointer to private structure.
 * @param idx
 *   TX queue index.
 *
 * @return
 *   A pointer to the queue if it exists.
 */
struct mlx5_txq_ctrl*
mlx5_priv_txq_get(struct mlx5_mdev_priv *priv, uint16_t idx)
{
	struct mlx5_txq_ctrl *ctrl = NULL;

	if ((*priv->txqs)[idx]) {
		ctrl = container_of((*priv->txqs)[idx], struct mlx5_txq_ctrl,
				    txq);
		unsigned int i;

		mlx5_priv_txq_mdev_get(priv, idx);
		for (i = 0; i != MLX5_PMD_TX_MP_CACHE; ++i) {
			struct mlx5_mdev_mr *mr;

			(void)mr;
			if (ctrl->txq.mp2mr[i]) {

				mr = priv_mr_get(priv, ctrl->txq.mp2mr[i]->mp);
				assert(mr);
			}
		}
		rte_atomic32_inc(&ctrl->refcnt);
		DEBUG("%p: Tx queue %p: refcnt %d", (void *)priv,
		      (void *)ctrl, rte_atomic32_read(&ctrl->refcnt));
	}
	return ctrl;
}

/**
 * Release a Tx queue.
 *
 * @param priv
 *   Pointer to private structure.
 * @param idx
 *   TX queue index.
 *
 * @return
 *   0 on success, errno on failure.
 */
int
mlx5_priv_txq_release(struct mlx5_mdev_priv *priv, uint16_t idx)
{
	unsigned int i;
	struct mlx5_txq_ctrl *txq;
	size_t page_size = sysconf(_SC_PAGESIZE);

	if (!(*priv->txqs)[idx])
		return 0;
	txq = container_of((*priv->txqs)[idx], struct mlx5_txq_ctrl, txq);
	DEBUG("%p: Tx queue %p: refcnt %d", (void *)priv,
	      (void *)txq, rte_atomic32_read(&txq->refcnt));
	if (txq->mdev) {
		int ret;

		ret = mlx5_priv_txq_mdev_release(priv, txq->mdev);
		if (!ret)
			txq->mdev = NULL;
	}
	for (i = 0; i != MLX5_PMD_TX_MP_CACHE; ++i) {
		if (txq->txq.mp2mr[i]) {
			priv_mr_release(priv, txq->txq.mp2mr[i]);
			txq->txq.mp2mr[i] = NULL;
		}
	}
	if (priv->uar_base)
		munmap((void *)RTE_ALIGN_FLOOR((uintptr_t)txq->txq.bf_reg,
		       page_size), page_size);
	if (rte_atomic32_dec_and_test(&txq->refcnt)) {
		txq_free_elts(txq);
		LIST_REMOVE(txq, next);
		rte_free(txq);
		(*priv->txqs)[idx] = NULL;
		return 0;
	}
	return EBUSY;
}

/**
 * Verify if the queue can be released.
 *
 * @param priv
 *   Pointer to private structure.
 * @param idx
 *   TX queue index.
 *
 * @return
 *   1 if the queue can be released.
 */
int
mlx5_priv_txq_releasable(struct mlx5_mdev_priv *priv, uint16_t idx)
{
	struct mlx5_txq_ctrl *txq;

	if (!(*priv->txqs)[idx])
		return -1;
	txq = container_of((*priv->txqs)[idx], struct mlx5_txq_ctrl, txq);
	return (rte_atomic32_read(&txq->refcnt) == 1);
}

/**
 * Verify the Tx Queue list is empty
 *
 * @param priv
 *  Pointer to private structure.
 *
 * @return the number of object not released.
 */
int
mlx5_priv_txq_verify(struct mlx5_mdev_priv *priv)
{
	struct mlx5_txq_ctrl *txq;
	int ret = 0;

	LIST_FOREACH(txq, &priv->txqsctrl, next) {
		DEBUG("%p: Tx Queue %p still referenced", (void *)priv,
		      (void *)txq);
		++ret;
	}
	return ret;
}
