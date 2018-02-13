/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2015 Mellanox.
 */

#include <stddef.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <net/if.h>
#include <math.h>

#include <rte_io.h>
#include <rte_pci.h>
#include <rte_ethdev_pci.h>
#include <rte_ether.h>
#include <rte_ethdev.h>
#include <rte_eth_ctrl.h>

#include "mdev_prm.h"
#include "mlx5_mdev_priv.h"

static int
mdev_priv_create_eq(struct mlx5_mdev_context *ctx __rte_unused, struct mdev_eq *eq)
{
	void *cqc;
	uint32_t in[MLX5_ST_SZ_DW(create_eq_in)] = {0};
	uint32_t out[MLX5_ST_SZ_DW(create_eq_out)];
	int err, status, syndrome;

	cqc = MLX5_ADDR_OF(create_eq_in, in, ctx);
	MLX5_SET(create_eq_in, in, opcode, MLX5_CMD_OP_CREATE_EQ);
	MLX5_ARRAY_SET64(create_eq_in, in, pas, 0,eq->buf->iova);

	MLX5_SET(eqc, cqc, log_eq_size, log2(eq->neqe));
	MLX5_SET(eqc, cqc, uar_page, eq->uar_page);
	MLX5_SET(eqc, cqc, log_page_size, log2(eq->buf->len /4096)); // TODO: from where ? MTT ???

	err = mlx5_mdev_cmd_exec(eq->ctx, in, sizeof(in), out, sizeof(out));
	if (err)
		return (err);
	status = MLX5_GET(create_eq_out, out, status);
	syndrome = MLX5_GET(create_eq_out, out, syndrome);
	eq->eqn = MLX5_GET(create_eq_out, out, eqn);

	printf("mdev_priv_create_eq status %x, syndrome = %x\n",status, syndrome);

	return status;
}

#define cqe_sz_to_mlx_sz(size) \
	(size)== 64 ? 0 : 1


static int
mdev_priv_create_cq(struct mlx5_mdev_context *ctx __rte_unused, struct mdev_cq *cq)
{
	void *cqc;
	uint32_t in[MLX5_ST_SZ_DW(create_cq_in)] = {0};
	uint32_t out[MLX5_ST_SZ_DW(create_cq_out)];
	int err, status, syndrome;

	cqc = MLX5_ADDR_OF(create_cq_in, in, ctx);
	MLX5_SET(create_cq_in, in, opcode, MLX5_CMD_OP_CREATE_CQ);
	MLX5_ARRAY_SET64(create_cq_in, in, pas, 0, cq->buf->iova);
	MLX5_SET(cqc, cqc, c_eqn, cq->eqn);
	MLX5_SET(cqc, cqc, cqe_sz, cqe_sz_to_mlx_sz(cq->cqe_size));
	MLX5_SET(cqc, cqc, uar_page, cq->uar_page);
	MLX5_SET(cqc, cqc, log_page_size, log2(cq->buf->len /4096)); // TODO: from where ? MTT ???
	MLX5_SET64(cqc, cqc, dbr_addr, cq->dbrec);   // FIXME
	MLX5_SET(cqc, cqc, log_cq_size, log2(cq->ncqe)); // WAS: cq->buf->len
	MLX5_SET(cqc, cqc, oi, 0);
	printf("mdev_priv_create_cq uar %x, dbrec = %lx\n",cq->uar_page, cq->dbrec);
	err = mlx5_mdev_cmd_exec(cq->ctx, in, sizeof(in), out, sizeof(out));
	if (err)
		return (err);
	cq->cqn = MLX5_GET(create_cq_out, out, cqn);
	cq->cons_index = 0;
	// cq->arm_sn     = 0;
	status = MLX5_GET(create_cq_out, out, status);
	syndrome = MLX5_GET(create_cq_out, out, syndrome);
	printf("mdev_priv_create_cq status %x, syndrome = %x\n",status, syndrome);

	return 0;
}


struct mdev_eq *
mlx5_mdev_create_eq(struct mlx5_mdev_priv *priv,
		    struct mdev_eq_attr *eq_attr)
{
	uint32_t eqe_size = 64;
	int log_max_cq_sz = 24; // TODO take from QUERY_HCA_CAP
	struct mdev_eq *eq;
	struct mlx5_mdev_context *ctx = priv->dev_context;
	uint32_t neqe, eq_size;
	int ret;

	if (!eq_attr->eqe) {
		return NULL;
	}
	eq = rte_zmalloc("mdev_eq", sizeof(*eq), ctx->cache_line_size); // TODO: make it numa node aware ?
	if(!eq)
		return NULL;
	neqe = 1UL << log2above(eq_attr->eqe + 1);
	eq_size = neqe * eqe_size;
	if ((neqe > 1UL << log_max_cq_sz) ||
	    (neqe < (eq_attr->eqe + 1))) {
		goto err_spl;
	}
	eq->ctx = ctx;
	eq->neqe = neqe;
	eq->buf = rte_eth_dma_zone_reserve(ctx->owner, "eq_buffer", 0, eq_size, eq_size,
						priv->edev->data->numa_node);
	if (!eq->buf)
		goto err_spl;

	eq->uar_page = ctx->uar;

	ret = mdev_priv_create_eq(ctx, eq);
	printf("create eq res == %d\n", ret);
	if (ret)
		goto err_ccq;
	return eq;
err_ccq:
	//mlx5_mdev_dealloc_uar(ctx, eq->uar_page); // fixme : remove
err_spl:
	//if (eq->buf)
	//	mdev_dealloc_cq_buf(ctx, eq->buf);
	//if (eq->dbrec)
	//	mlx5_return_dbrec(priv, eq->dbrec);
	//rte_free(eq);
	return NULL;
}

struct mdev_cq *
mlx5_mdev_create_cq(struct mlx5_mdev_priv *priv,
		    struct mdev_cq_attr *cq_attr)
{
	uint32_t cqe_size = 64; // TODO make it a user parameter ?
	struct mdev_cq *cq;
	struct mlx5_mdev_context *ctx = priv->dev_context;
	uint32_t ncqe, cq_size;
	int ret;


	if (!cq_attr->cqe) {
		return NULL;
	}
	cq = rte_zmalloc("mdev_cq", sizeof(*cq), ctx->cache_line_size); // TODO: make it numa node aware ?
	if(!cq)
		return NULL;
	ncqe = 1UL << log2above(cq_attr->cqe + 1);
	cq_size = ncqe * cqe_size;
	cq->dbrec = mlx5_get_dbrec(priv);
	if (!cq->dbrec)
		goto err_spl;
	cq->uar_page = ctx->uar;
	/* Fill info for create CQ */
	cq->eqn = cq_attr->eqn;
	cq->buf = rte_eth_dma_zone_reserve(ctx->owner, "cq_buffer", 0, cq_size, cq_size,
						priv->edev->data->numa_node);
	cq->ctx = cq_attr->ctx;
	ret = mdev_priv_create_cq(ctx, cq);
	if (ret)
		goto err_ccq;
	printf("create CQ res == %d\n",ret);
	return cq;
err_ccq:
	//mlx5_mdev_dealloc_uar(ctx, cq->uar_page); // fixme : remove
err_spl:
	//if (cq->buf)
	//	mdev_dealloc_cq_buf(ctx, cq->buf);
	//if (cq->dbrec)
	//	mlx5_return_dbrec(priv, cq->dbrec);
	//rte_free(cq);
	return NULL;
}

