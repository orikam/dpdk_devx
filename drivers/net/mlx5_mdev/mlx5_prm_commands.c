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

#include "mlx5_mdev_utils.h"
#include "mdev_prm.h"
#include "mlx5_mdev_priv.h"
#include "devx.h"

static int
mdev_priv_create_eq(void *dev __rte_unused, struct mdev_eq *eq)
{
	void *cqc;
	uint32_t in[MLX5_ST_SZ_DW(create_eq_in)] = {0};
	uint32_t out[MLX5_ST_SZ_DW(create_eq_out)];
	int status, syndrome;

	cqc = MLX5_ADDR_OF(create_eq_in, in, ctx);
	MLX5_SET(create_eq_in, in, opcode, MLX5_CMD_OP_CREATE_EQ);
	MLX5_ARRAY_SET64(create_eq_in, in, pas, 0,eq->buf->iova);

	MLX5_SET(eqc, cqc, log_eq_size, log2(eq->neqe));
	MLX5_SET(eqc, cqc, uar_page, eq->uar_page);
	MLX5_SET(eqc, cqc, log_page_size, log2(eq->buf->len /4096)); // TODO: from where ? MTT ???

	eq->object = devx_obj_create(eq->dev, in, sizeof(in), out, sizeof(out));
	if (eq->object)
		return 1;
	status = MLX5_GET(create_eq_out, out, status);
	syndrome = MLX5_GET(create_eq_out, out, syndrome);
	eq->eqn = MLX5_GET(create_eq_out, out, eqn);

	printf("mdev_priv_create_eq status %x, syndrome = %x\n",status, syndrome);

	return status;
}

#define cqe_sz_to_mlx_sz(size) \
	(size)== 64 ? 0 : 1


static int
mdev_priv_create_cq(void *dev __rte_unused, struct mdev_cq *cq)
{
	void *cqc;
	uint32_t in[MLX5_ST_SZ_DW(create_cq_in)] = {0};
	uint32_t out[MLX5_ST_SZ_DW(create_cq_out)];
	int status, syndrome;

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
	cq->object = devx_obj_create(cq->dev, in, sizeof(in), out, sizeof(out));
	if (cq->object)
		return 1;
	cq->cqn = MLX5_GET(create_cq_out, out, cqn);
	cq->cons_index = 0;
	// cq->arm_sn     = 0;
	status = MLX5_GET(create_cq_out, out, status);
	syndrome = MLX5_GET(create_cq_out, out, syndrome);
	printf("mdev_priv_create_cq status %x, syndrome = %x\n",status, syndrome);

	return 0;
}


static int
mdev_priv_create_tis(void *dev __rte_unused, struct mdev_tis *tis)
{
	void *tisc;
	uint32_t in[MLX5_ST_SZ_DW(create_tis_in)] = {0};
	uint32_t out[MLX5_ST_SZ_DW(create_tis_out)];
	int status, syndrome;

	tisc = MLX5_ADDR_OF(create_tis_in, in, ctx);
	MLX5_SET(create_tis_in, in, opcode, MLX5_CMD_OP_CREATE_TIS);
	MLX5_SET(tisc, tisc, prio, (tis->priority)<<1);
	MLX5_SET(tisc, tisc, transport_domain, tis->td);
	tis->object = devx_obj_create(tis->dev, in, sizeof(in), out, sizeof(out));
	if (tis->object)
		return 1;

	status = MLX5_GET(create_cq_out, out, status);
	syndrome = MLX5_GET(create_cq_out, out, syndrome);
	if(!status)
		tis->tisn = MLX5_GET(create_tis_out, out, tisn);
	printf("mdev_priv_create_tis status %x, syndrome = %x\n",status, syndrome);

	return status;
}

static int
mdev_priv_create_sq(void *dev __rte_unused, struct mdev_sq *sq)
{
	void *sqc;
	void *wqc;
	uint32_t in[MLX5_ST_SZ_DW(create_sq_in)] = {0};
	uint32_t out[MLX5_ST_SZ_DW(create_sq_out)] = {0};
	int status, syndrome;

	MLX5_SET(create_cq_in, in, opcode, MLX5_CMD_OP_CREATE_SQ);

	sqc = MLX5_ADDR_OF(create_sq_in, in, ctx);
	MLX5_SET(sqc, sqc, cqn, sq->cqn);
	MLX5_SET(sqc, sqc, tis_lst_sz, 1);
	MLX5_SET(sqc, sqc, tis_num_0, sq->tisn);

	wqc = MLX5_ADDR_OF(sqc, sqc, wq);
	MLX5_SET(wq, wqc, wq_type, 0x1);
	MLX5_SET(wq, wqc, pd, sq->wq.pd);
	MLX5_SET64(wq, wqc, dbr_addr, sq->wq.dbr_addr);
	MLX5_SET(wq, wqc, log_wq_stride, 6);
	MLX5_SET(wq, wqc, log_wq_pg_sz, log2(sq->wq.buf->len /4096));
	MLX5_SET(wq, wqc, log_wq_sz, log2(sq->wq.buf->len>>6));
	MLX5_ARRAY_SET64(wq, wqc, pas, 0, sq->wq.buf->iova);

	sq->object = devx_obj_create(sq->dev, in, sizeof(in), out, sizeof(out));
	if (sq->object)
		return 1;
	status = MLX5_GET(create_sq_out, out, status);
	syndrome = MLX5_GET(create_sq_out, out, syndrome);
	printf("mdev_priv_create_sq status %x, syndrome = %x\n",status, syndrome);

	return status;
}

struct mdev_eq *
mlx5_mdev_create_eq(struct mlx5_mdev_priv *priv,
		    struct mdev_eq_attr *eq_attr)
{
	uint32_t eqe_size = 64;
	int log_max_cq_sz = 24; // TODO take from QUERY_HCA_CAP
	struct mdev_eq *eq;
	void *dev = priv->dev;
	uint32_t neqe, eq_size;
	int ret;

	if (!eq_attr->eqe) {
		return NULL;
	}
	eq = rte_zmalloc("mdev_eq", sizeof(*eq), priv->cache_line_size); // TODO: make it numa node aware ?
	if(!eq)
		return NULL;
	neqe = 1UL << log2above(eq_attr->eqe + 1);
	eq_size = neqe * eqe_size;
	if ((neqe > 1UL << log_max_cq_sz) ||
	    (neqe < (eq_attr->eqe + 1))) {
		goto err_spl;
	}
	//eq->ctx = ctx;
	eq->neqe = neqe;
	eq->buf = rte_eth_dma_zone_reserve(priv->dev, "eq_buffer", 0, eq_size, eq_size,
						priv->dev->data->numa_node);
	if (!eq->buf)
		goto err_spl;

	eq->uar_page = eq_attr->uar;

	ret = mdev_priv_create_eq(dev, eq);
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
	void *dev = priv->dev;
	uint32_t ncqe, cq_size;
	int ret;


	if (!cq_attr->cqe) {
		return NULL;
	}
	cq = rte_zmalloc("mdev_cq", sizeof(*cq), 64); // TODO: make it numa node aware ?
	if(!cq)
		return NULL;
	ncqe = 1UL << log2above(cq_attr->cqe + 1);
	cq_size = ncqe * cqe_size;
	cq->dbrec = mlx5_get_dbrec(priv);
	if (!cq->dbrec)
		goto err_spl;
	//cq->uar_page = ctx->uar;
	/* Fill info for create CQ */
	cq->eqn = cq_attr->eqn;
	cq->buf = rte_eth_dma_zone_reserve(priv->dev, "cq_buffer", 0, cq_size, cq_size,
						priv->dev->data->numa_node);
	cq->dev = dev;
	ret = mdev_priv_create_cq(dev, cq);
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

struct mdev_tis *
mlx5_mdev_create_tis(struct mlx5_mdev_priv *priv,
		    struct mdev_tis_attr *tis_attr)
{
	void *dev = priv->dev;
	int ret;
	struct mdev_tis *tis;

	if (!tis_attr->td) {
		return NULL;
	}
	tis = rte_zmalloc("tis", sizeof(*tis), priv->cache_line_size);
	if(!tis)
		return NULL;

	tis->dev = dev;
	tis->td = tis_attr->td;

	ret = mdev_priv_create_tis(dev, tis);
	printf("create tis res == %d\n", ret);
	if (ret)
		goto err_tis;
	return tis;
err_tis:

	return NULL;
}


struct mdev_sq *
mlx5_mdev_create_sq(struct mlx5_mdev_priv *priv,
		    struct mdev_sq_attr *sq_attr)
{
	void *dev = priv->dev;
	int ret;
	struct mdev_sq *sq;


	sq = rte_zmalloc("sq", sizeof(*sq), priv->cache_line_size);
	if(!sq)
		return NULL;

	sq->dev = dev;
	sq->wq.pd = sq_attr->wq.pd;
	sq->cqn = sq_attr->cqn;
	sq->tisn = sq_attr->tisn;
	sq->wq.dbr_addr = mlx5_get_dbrec(priv);
	sq->wq.uar_page = sq_attr->uar;
	sq->wq.buf = rte_eth_dma_zone_reserve(priv->dev, "sq_buffer", 0, sq_attr->nelements * 64, 4096,
							priv->dev->data->numa_node);
	ret = mdev_priv_create_sq(dev, sq);
	printf("create sq res == %d\n", ret);
	if (ret)
		goto err_sq;
	return sq;
err_sq:

	return NULL;
}

int mlx5_mdev_alloc_pd(struct mlx5_mdev_priv *priv)
{
	uint32_t in[MLX5_ST_SZ_DW(alloc_pd_in)]   = {0};
	uint32_t out[MLX5_ST_SZ_DW(alloc_pd_out)] = {0};
	uint32_t status;

	MLX5_SET(alloc_pd_in, in, opcode, MLX5_CMD_OP_ALLOC_PD);
	priv->pd.pd_object = devx_obj_create(priv->ctx, in, sizeof(in), out, sizeof(out));
	if (!priv->pd.pd_object)
		return 1;

	status = MLX5_GET(alloc_pd_out, out, status);
	priv->pd.pd = MLX5_GET(alloc_pd_out, out, pd);
	printf("oooOri pd status %d pd value %d\n",status, priv->pd.pd);
	return status;
}

int mlx5_mdev_alloc_td(struct mlx5_mdev_priv *priv)
{
	uint32_t in[MLX5_ST_SZ_DW(alloc_transport_domain_in)]   = {0};
	uint32_t out[MLX5_ST_SZ_DW(alloc_transport_domain_out)] = {0};
	uint32_t status;


	MLX5_SET(alloc_transport_domain_in, in, opcode,
		 MLX5_CMD_OP_ALLOC_TRANSPORT_DOMAIN);
	priv->td.td_object = devx_obj_create(priv->ctx, in, sizeof(in), out, sizeof(out));
	if (!priv->td.td_object)
		return 1;

	priv->td.td = MLX5_GET(alloc_transport_domain_out, out, transport_domain);

	status = MLX5_GET(alloc_pd_out, out, status);
	printf("oooOri td status %d td value %d\n",status, priv->td.td);
	return status;
}
